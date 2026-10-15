// Street fidelity P2 slice 9 (design sections 5.3, 6.1-6.4): the carved terrain layer g2m.terrain.carved.
//
// Level 0 is the bare g2m.elev.base terrain, smoothed by the ported game kernel and then carved by the served road
// geometry: junction cores (the Shepard junction surface of 4.2), approach cores (road_surface_height_m of 4.1), blends
// of 3-15 m widened to the 1:1.5 slope limit, and last the deck clearance clamp under bridges. Levels 1-6 are the
// mean-of-4 half-even pyramid of the carved level 0 (downsample_2x2, as g2m.terrain.height). g2m.terrain.height stays
// unchanged and the default; a consumer opts in by requesting this layer.
//
// Every cell is a function of its world position: a level-0 tile Q reads its own and its 8 neighbouring elev.base tiles
// (smoothing and the blend widths read the DEM up to 17 m outside Q) and every roads.geom level-2 tile within 192 m of
// Q (design 5.3: every junction that can touch Q has its owner node within 191 m), so neighbouring tiles agree at their
// shared edge without sharing samples. Order of one cell (design 6.2-6.4): smoothing, ownership (junction cores beat
// approach cores, lowest core wins, then the blend of largest weight, ties to the lowest z, else the smoothed terrain),
// the deck clamp, then round_half_even(z * 256). Bridges and tunnels are never carved; OnStructure junctions are not
// carved. Only + - * / and the det helpers; no unordered iteration decides a value.
#pragma once
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/carve_params.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_junction.h"
#include "g2m/layer/road_rules.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
inline constexpr std::string_view kTerrainCarvedLayer = "g2m.terrain.carved";
// The roads.geom search radius around a level-0 tile (design 5.3).
inline constexpr std::int64_t kCarveRoadRadiusM = 192;
// The bucket size of the carve model (design 6.2).
inline constexpr std::int64_t kCarveBucketM = 16;
// The model covers the tile grown by this many cells: blends reach 15 m outside a core, the smoothed DEM is read at
// road edge points up to ~17 m from a cell, and the tile-invariance tests evaluate a few cells past each edge.
inline constexpr std::int64_t kCarveMarginCells = 32;

// Smoothed bare terrain over a level-0 tile grown by `margin` cells, from the 3 x 3 elev.base window (design 6.2 step 1,
// the game's terrain smoothing as an integer kernel): per axis (x, then y) sum(k_i * h_i) / sum(k_i) over the taps that
// are not NoData, rounded half-even (int64); then h + round_half_even(strength * (s - h) / 100). A NoData cell stays
// NoData. Disabled smoothing returns the raw heights. Heights are 1/256 m units with the input header offsets applied.
class CarveTerrain {
public:
    // `window[(dy + 1) * 3 + (dx + 1)]` is the level-0 tile at (centre.x + dx, centre.y + dy), nullptr when absent
    // (treated as NoData). Every present tile must carry its own key.
    static Result<CarveTerrain> make(const TileKey& centre, std::span<const HeightTile* const> window,
        const CarveSmoothing& smoothing, std::int64_t margin = kCarveMarginCells);
    // Height units of the world cell whose south-west corner is (e, n) in zone metres; kHeightNoData where the DEM has
    // no data. raw() covers the whole 3 x 3 window (NoData outside it). smoothed() reads a table over the covered
    // region and computes any other cell on demand with the same function (D37: a junction ring can reach road-side
    // points far outside the covered region); a cell whose kernel leaves the window sees those taps as NoData, so a
    // caller that needs a tile-invariant value checks in_window() first.
    std::int32_t smoothed(std::int64_t e, std::int64_t n) const noexcept;
    std::int32_t raw(std::int64_t e, std::int64_t n) const noexcept;
    bool in_window(std::int64_t e, std::int64_t n) const noexcept;   // the whole kernel of cell (e, n) lies in the window
    const TileKey& centre() const noexcept { return centre_; }
    std::int64_t e0() const noexcept { return e0_; }   // south-west cell of the covered region
    std::int64_t n0() const noexcept { return n0_; }
    std::int64_t size() const noexcept { return size_; }
private:
    std::int32_t smooth_cell(std::int64_t e, std::int64_t n) const noexcept;
    TileKey centre_;
    CarveSmoothing smoothing_;
    std::int64_t e0_ = 0, n0_ = 0, size_ = 0;      // the smoothed table (covered region)
    std::int64_t we0_ = 0, wn0_ = 0;               // south-west cell of the 768 x 768 window
    std::vector<std::int32_t> raw_, smoothed_;
};

// Switches for the tests' sabotages and negative controls. Production uses the defaults.
struct CarveOptions {
    bool blend_from_raw = false;            // T12 sabotage: blends (and their widths) read the raw DEM, the terrain the smoothed one
    bool first_come_ownership = false;      // T14 sabotage: the first candidate of each kind wins instead of the min / max rules
    bool deck_clamp = true;                 // T15 sabotage: no clearance clamp under bridges
    bool junction_relief_fade = true;       // D42 sabotage: false keeps every approach's full relief (the D22-D24 surface)
    std::uint64_t candidate_order_seed = 0; // T14: visit the candidates of a bucket in a seeded permutation (0 = stored order)
};

enum class CarveOwner : std::uint8_t { NoData = 0, Terrain = 1, JunctionCore = 2, ApproachCore = 3, JunctionBlend = 4, ApproachBlend = 5 };

// One evaluation of the model at a point.
struct CarveSample {
    double z = 0;                       // metres; NaN for NoData
    CarveOwner owner = CarveOwner::NoData;
    std::size_t element = 0;            // the owner: a carved junction (CarveModel::junction(element)) or a merged entry index
    double weight = 0;                  // the blend weight w (1 for a core)
    double terrain = 0;                 // the smoothed terrain T at the point's cell (metres)
    bool deck_clamped = false;          // the deck clamp lowered z
};

// Counters of one carve (diagnostics and the G3-D tables; never stored).
struct CarveStats {
    std::size_t entries = 0, ground_entries = 0, bridge_entries = 0, tunnel_entries = 0;
    std::size_t junctions = 0;                  // loaded records (unique by owner)
    std::size_t junctions_carved = 0, junctions_declined = 0, junctions_on_structure = 0, junctions_no_surface = 0;
    std::size_t cells_terrain = 0, cells_nodata = 0, cells_junction_core = 0, cells_approach_core = 0;
    std::size_t cells_junction_blend = 0, cells_approach_blend = 0;
    std::size_t cells_deck_clamped = 0;
    std::size_t deck_clearance_violations = 0;  // a core cell the clamp lowered (clearance < deck clearance: a profile problem)
    std::size_t blend_stations = 0;             // blend widths computed at stations whose edge point lies in the tile
    std::size_t steep_blend_stations = 0;       // ... with factor * |T_edge - z_edge| > blend_max: steeper than 1:1.5 (counted)
    double max_blend_width_m = 0;
    std::size_t junction_knots_outside_window = 0;   // D37: a junction blend knot whose DEM point lies outside the 3 x 3 window (b = b_class)
    void add_cells(const CarveStats& o);        // adds the cell counters of `o`
};

// One decoded roads.geom level-2 tile.
struct CarveGeomInput {
    TileKey key;
    const RoadGeomTile* tile = nullptr;
};

// The carve model of one level-0 tile (design 6.2 step 2): the union of the loaded tiles' junctions (unique by owner
// node and record kind) and entries (unique by way id and stretch; the copy from the lowest tile key wins, P1
// invariance makes the copies equal), the approach features adjusted for those junctions (D23/D34), the approach
// polylines (1 m stations, 0.5 m within 2 m of a reference segment joint), the blend widths per station and side, and
// 16 m buckets over the covered region. evaluate() is pure and allocation-free.
class CarveModel {
public:
    // `terrain` must outlive the model. `inputs` in any order (they are sorted by key).
    static Result<CarveModel> build(std::span<const CarveGeomInput> inputs, const CarveParams& params,
        const CarveTerrain& terrain, const CarveOptions& options = {}, const RoadRules& rules = builtin_road_rules_de(),
        CarveStats* stats = nullptr);
    CarveModel(CarveModel&&) noexcept;
    CarveModel& operator=(CarveModel&&) noexcept;
    ~CarveModel();
    // The carved height at (x, y) in zone metres (a cell centre is (e + 0.5, n + 0.5)); the terrain T is the smoothed
    // DEM of the cell containing the point. Points outside the covered region return the terrain only.
    CarveSample evaluate(double x, double y) const noexcept;
    // Test hooks.
    const RoadGeomTile& core_tile() const noexcept;     // merged entries, features incl. the junction ramps (approach cores)
    const RoadGeomTile& kept_tile() const noexcept;     // merged entries, features without suppressed ones and ramps (junctions)
    std::size_t junction_count() const noexcept;        // carved junctions
    const RoadJunction& junction(std::size_t i) const noexcept;   // entry indices refer to the merged tiles
    const JunctionSurface& junction_surface(std::size_t i) const noexcept;
    double trim_start(std::size_t entry) const noexcept;   // 0 without a loaded junction at that end
    double trim_end(std::size_t entry) const noexcept;
    // The blend width at station s on one side of a carved ground entry (NaN when the station has none).
    double blend_width_m(std::size_t entry, double s, bool left) const noexcept;
    // The model's own projection of (x, y) onto a ground or bridge entry's polyline (false outside it or without a line).
    bool station_of(std::size_t entry, double x, double y, double& s, double& t) const;
private:
    struct Impl;
    explicit CarveModel(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

// The carved level-0 tile `key` (design 6.2): smoothing, model, every cell, deck clamp, quantisation. `dem` is the 3 x 3
// window of CarveTerrain::make. `workers` > 1 splits the rows over threads; the result is identical for every count.
Result<HeightTile> carve_terrain_tile(const TileKey& key, std::span<const HeightTile* const> dem,
    std::span<const CarveGeomInput> roads, const CarveParams& params = builtin_carve_default(), const CarveOptions& options = {},
    unsigned workers = 1, CarveStats* stats = nullptr);

// The roads.geom level-2 keys a level-0 tile reads: every level-2 tile intersecting the tile's square grown by 192 m
// (half-open), ascending. One typically, at most four.
std::vector<TileKey> carve_road_geom_keys(const TileKey& level0);

LayerDesc terrain_carved_desc();
// The text TerrainCarvedDeriver hashes into its config hash (design 9.3).
std::string terrain_carved_config_text();

// g2m.terrain.carved. L0: inputs = the 3 x 3 g2m.elev.base neighbourhood (absent = NoData; an absent centre is
// NotFound, as g2m.terrain.height) and carve_road_geom_keys (absent = no roads there). Offsets are applied once; the
// output offset is 0 and the datum is the DEM's. L1-L6: the 4 same-layer children, mean-of-4 half-even (a missing
// child counts as NoData). Coverage follows g2m.elev.base.
class TerrainCarvedDeriver final : public IDeriver {
public:
    TerrainCarvedDeriver();
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::span<const std::string> coverage_inputs() const override { return std::span<const std::string>(desc_.inputs).first(1); }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
} // namespace g2m
