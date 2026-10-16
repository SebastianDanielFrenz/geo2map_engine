// g2m/world/view_streamer.h - render LOD that follows a moving focus point (terrain migration plan phase 3b).
//
// Ported from racing_game core/src/world_terrain.cpp (select_view_keys, build_render_chunks, build_one_chunk,
// chunk_window_carve), core/src/terrain_view_streamer.cpp (TerrainViewStreamer), core/src/terrain_stream.cpp
// (reselect_distance_for_speed, chunk_time_to_reach_s, sort_adds_by_urgency) and core/src/lod_settings.cpp
// (changed_road_lod_levels), 5a3baed, statement for statement with det math (std::hypot -> det::hypot).
// tests/world/test_view_streamer.cpp pins the output against the game's own code.
//
// Engine-neutral: a chunk is geo2map's TerrainChunkMesh plus its key, carve state and session-local origin; the
// per-vertex road/land class ids are the mesh's own `land_class`. Colours are the consumer's: ViewSource::colour
// (optional) fills ViewChunk::rgba from those class ids and the mesh, on the build thread.
//
// ---------------------------------------------------------------------------
// Adapter ordering contract of ViewStreamer (no holes):
//
//   every frame:
//     1. streamer.update_focus(x, y)             never blocks; may start a job
//     2. if no diff is in progress:
//          streamer.poll(diff)                   never blocks; true = a diff is yours now (phase Applying)
//     3. upload diff.added within this frame's budget (any order, any number per frame)
//     4. ONLY once the LAST add of this diff has been uploaded: free every chunk in diff.removed, then call
//        streamer.commit(diff.serial)
//
//   Removals after adds: the view only ever holds old + (part of new) and then new, never less than one complete
//   selection. While a diff is Ready or Applying no new selection starts; update_focus() only records the latest
//   focus and commit() starts one selection for the LATEST focus if it is due (moves while busy coalesce). An empty
//   diff (the selection did not change) is still delivered; the adapter just commits it. diff.replaced (a road LOD
//   change): swap each in place of the resident chunk with the same key (upload the new one, free the old) before
//   commit().
//
// Threading: every public ViewStreamer method is called from ONE adapter thread; the class owns one worker thread
// (selection + build, fanned out to Options::build_threads short-lived threads). The source's lookup, class lookup,
// carve function and colour hook must be thread-safe. The destructor raises a cancel flag (checked before every chunk)
// and joins the worker.
// ---------------------------------------------------------------------------
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/mesh/lod_select.h"
#include "g2m/mesh/terrain_chunk.h"
#include "g2m/world/chunk_road_classes.h"
#include "g2m/world/chunk_terrain_classes.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace g2m::world {

// How the height tile(s) a chunk was built from were carved with the road surface. A chunk built from a fallback
// tile (Fallback: produced uncarved because the road surface was missing) shows ground above the carved road bed;
// the consumer rebuilds it once the tile is carved.
enum class ChunkCarve : std::uint8_t { NotCarved = 0, Carved = 1, Fallback = 2 };

// The carve state of one height tile (a ChunkCarve value as uint8_t); `ctx` is ViewSource::ctx.
using TileCarveFn = std::uint8_t (*)(void* ctx, const TileKey&);

// One built chunk. `mesh` local to mesh.origin (absolute UTM); origin_session = mesh.origin - (e0, n0, 0).
// `rgba`: what ViewSource::colour wrote (one packed colour per vertex by the consumer's convention), empty without one.
struct ViewChunk {
    mesh::ChunkKey key{};
    // The WORST carve state of the chunk's height window (any Fallback tile = Fallback).
    ChunkCarve carve = ChunkCarve::NotCarved;
    mesh::TerrainChunkMesh mesh;
    double origin_session[3] = {0.0, 0.0, 0.0};
    std::vector<std::uint32_t> rgba;
};

// Fills chunk.rgba (and may recolour) after the mesh, key, carve and origin are set. Runs on build threads.
using ChunkColourFn = std::function<void(ViewChunk&)>;

// Everything a selection + build needs. `lookup(ctx, key)` must be thread-safe and `ctx` must outlive every holder.
struct ViewSource {
    mesh::LodParams params;
    double e0 = 0.0; // session origin, absolute UTM
    double n0 = 0.0;
    mesh::TileLookup lookup = nullptr;
    void* ctx = nullptr;
    // Null fn = every chunk built without road classes (build_chunk with no ClassWindow).
    ClassLookup class_lookup{};
    // The served alternative (P2 slice 11, D62): a non-null fn takes precedence over class_lookup, and the window is
    // read from g2m.terrain.class (chunk_terrain_classes.h). Its `lod` follows the same convention as
    // class_lookup.lod (point it at road_lod).
    TerrainClassLookup terrain_classes{};
    // Null = every chunk reports NotCarved; called with `ctx`.
    TileCarveFn carve_state = nullptr;
    // Optional colour hook (see ChunkColourFn).
    ChunkColourFn colour;
    // The render road LOD policy class_lookup.lod points at; holding it here keeps that pointer valid for every copy.
    // Null = default_render_road_lod().
    std::shared_ptr<const RenderRoadLod> road_lod;
};

// The worst carve state over the height tiles a chunk's halo window touches (gather_window: chunk samples
// 64*c - 1 .. 64*c + 65 of its level, 256 samples per tile).
ChunkCarve chunk_window_carve(const mesh::ChunkKey& key, void* ctx, TileCarveFn fn);

// select_chunks around the session-local point (cam_x, cam_y): `out` in select_chunks' order (level desc, cy, cx).
void select_view_keys(double cam_x, double cam_y, const mesh::LodParams& params, double e0, double n0,
                      std::vector<mesh::ChunkKey>& out);

// One ViewChunk per key (out[i] <-> keys[i]) with up to `thread_count` workers, each writing only its own indices:
// byte-identical for any thread_count >= 1. `cancel` (optional) is checked before each chunk; once true the rest is
// skipped and false returned (`out` partial, discard it). source.params is not used (the keys are given).
bool build_view_chunks(const std::vector<mesh::ChunkKey>& keys, const ViewSource& source, unsigned thread_count,
                       std::vector<ViewChunk>& out, const std::atomic<bool>* cancel = nullptr);

// select_view_keys + build_view_chunks (select_chunks order).
void build_static_view(double cam_x, double cam_y, const ViewSource& source, unsigned thread_count,
                       std::vector<ViewChunk>& out);

// Reselect distance at speed: the base distance shrunk linearly to `min_fraction` of it between speed_lo_mps and
// speed_hi_mps; speed 0 (no velocity) = the base distance.
[[nodiscard]] double reselect_distance_for_speed(double base_m, double speed_mps, double min_fraction = 0.5,
                                                 double speed_lo_mps = 40.0, double speed_hi_mps = 100.0);

// Seconds until a car at (px, py) (session-local) moving with (vx, vy) reaches the chunk's footprint; behind the car
// 1000 s plus its distance; below 2 m/s the plain distance to the footprint over 2 m/s.
[[nodiscard]] double chunk_time_to_reach_s(const mesh::ChunkKey& key, double e0, double n0, double px, double py,
                                           double vx, double vy);

// Upload order of `added`: time bins of 0.25 s, then finer level first, then time, then key.
void sort_adds_by_urgency(std::vector<ViewChunk>& added, double e0, double n0, double px, double py, double vx,
                          double vy);

// Chunk levels 0..max_level whose road-class content differs between two policies (null = default).
std::vector<int> changed_road_lod_levels(const RenderRoadLod* a, const RenderRoadLod* b, int max_level);

struct ViewDiff {
    std::uint64_t serial = 0; // pass to commit(); 1, 2, 3, ... per streamer
    double focus_x = 0.0;     // the session-local focus this diff selects for
    double focus_y = 0.0;
    double velocity_x = 0.0;  // the focus velocity the selection was made with (0 = none)
    double velocity_y = 0.0;
    std::chrono::steady_clock::time_point selected_at{}; // when the selection job started
    // new set minus resident set, built; sorted by key, or (speed >= 2 m/s) in sort_adds_by_urgency order.
    std::vector<ViewChunk> added;
    std::vector<mesh::ChunkKey> removed; // resident minus new, sorted
    // Resident chunks that stay but were rebuilt for a road LOD change at their level. Sorted by key.
    std::vector<ViewChunk> replaced;
};

class ViewStreamer {
public:
    struct Options {
        double reselect_distance_m = 128.0; // a selection is due once the focus is MORE than this from the last one
        unsigned build_threads = 2;         // threads the background build fans out to (>= 1)
        unsigned initial_build_threads = 0; // build_initial on the caller's thread (0 = min(hardware, 8))
    };

    enum class Phase { Idle, Building, Ready, Applying };

    struct Stats {
        std::uint64_t selections_started = 0;
        std::uint64_t diffs_committed = 0;
        std::uint64_t chunks_built = 0;  // by the worker (not build_initial)
        std::uint64_t chunks_reused = 0; // new-set keys already resident, summed over selections
        std::size_t last_added = 0;
        std::size_t last_removed = 0;
        std::uint64_t chunks_rebuilt = 0; // ViewDiff::replaced
        double last_build_ms = 0.0;
        std::size_t resident_count = 0;
    };

    ViewStreamer(ViewSource source, Options options);
    explicit ViewStreamer(ViewSource source) : ViewStreamer(std::move(source), Options{}) {}
    ~ViewStreamer();
    ViewStreamer(const ViewStreamer&) = delete;
    ViewStreamer& operator=(const ViewStreamer&) = delete;

    // Synchronous first selection around (x, y) on the calling thread (select_chunks order, = build_static_view);
    // records the keys as resident. Only while Idle (false otherwise, `out` untouched).
    bool build_initial(double x, double y, std::vector<ViewChunk>& out);

    // Records the latest focus; starts a background selection iff Idle and (no selection yet, or the focus is more
    // than reselect_distance_for_speed(...) from the last selection's focus, or set_source was called). (vx, vy): the
    // velocity (0 = unknown; >= 2 m/s orders the adds by urgency); (ref_x, ref_y): where the car is (NaN = the focus).
    void update_focus(double x, double y, double vx = 0.0, double vy = 0.0,
                      double ref_x = std::numeric_limits<double>::quiet_NaN(),
                      double ref_y = std::numeric_limits<double>::quiet_NaN());

    // New LodParams and/or road LOD policy: the next selection is due at once; resident chunks whose level's
    // render_road_lod_key changed are rebuilt into ViewDiff::replaced.
    void set_source(const ViewSource& source);

    // Non-blocking: a Ready diff into `out` (phase Applying), true; else false.
    bool poll(ViewDiff& out);

    // Contract step 4 done for diff `serial`: its selection becomes resident, Idle, and a due follow-up starts.
    bool commit(std::uint64_t serial);

    [[nodiscard]] Phase phase() const;
    [[nodiscard]] Stats stats() const;
    [[nodiscard]] std::vector<mesh::ChunkKey> resident_keys() const; // sorted copy

private:
    void worker_main();
    void maybe_start_locked();

    ViewSource source_;
    const Options options_;
    bool source_dirty_ = false;
    std::shared_ptr<const RenderRoadLod> resident_road_lod_;
    std::shared_ptr<const RenderRoadLod> target_road_lod_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    Phase phase_ = Phase::Idle;
    bool stop_ = false;
    bool job_pending_ = false;

    bool has_selection_ = false;
    double selection_x_ = 0.0;
    double selection_y_ = 0.0;
    double latest_x_ = 0.0;
    double latest_y_ = 0.0;
    double latest_vx_ = 0.0;
    double latest_vy_ = 0.0;
    double latest_rx_ = 0.0;
    double latest_ry_ = 0.0;
    bool has_latest_ = false;

    std::uint64_t next_serial_ = 1;
    std::uint64_t job_serial_ = 0;
    double job_x_ = 0.0;
    double job_y_ = 0.0;
    double job_vx_ = 0.0;
    double job_vy_ = 0.0;
    double job_rx_ = 0.0;
    double job_ry_ = 0.0;

    std::vector<mesh::ChunkKey> resident_;
    std::vector<mesh::ChunkKey> target_;
    ViewDiff ready_;
    std::uint64_t applying_serial_ = 0;
    Stats stats_;

    std::atomic<bool> cancel_{false};
    std::thread worker_; // last member
};

} // namespace g2m::world
