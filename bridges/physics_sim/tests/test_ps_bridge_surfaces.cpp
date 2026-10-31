// Phase 9 (terrain migration plan 6.6): bind_class_surfaces / bind_road_surfaces moved from racing_game's
// Session::setup_terrain. Parity: for the game's own surface table (fixtures/game_surfaces.json, a copy of
// racing_game data/surfaces/surfaces.json at 9fa3feb) the class and road tables must equal, entry by entry, what the
// game's inline blocks (resolve_grip_surface, the SurfaceIdTable::build lambdas, RoadValueLut::by_land_class) build.
// The reference below is that code transcribed unchanged (only the error handling is a throw, as in the game).
#include "g2m/ps_bridge/surface_binding.h"

#include "ps/io/surface_table.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#ifndef G2M_BRIDGE_FIXTURE_DIR
#error "G2M_BRIDGE_FIXTURE_DIR must name bridges/physics_sim/tests/fixtures"
#endif

using g2m::ps_bridge::bind_class_surfaces;
using g2m::ps_bridge::bind_road_surfaces;
using g2m::ps_bridge::SurfaceNames;

namespace {

// racing_game core/include/rg/surface_kinds.h, surface_name_for_kind (rail_bridges 9506513), unchanged.
std::string game_surface_name_for_kind(g2m::SurfaceKind kind) {
    switch (kind) {
        case g2m::SurfaceKind::Asphalt: return "asphalt";
        case g2m::SurfaceKind::Concrete: return "concrete";
        case g2m::SurfaceKind::PavingStones: return "paving_stones";
        case g2m::SurfaceKind::Sett: return "sett";
        case g2m::SurfaceKind::Cobblestone: return "cobblestone";
        case g2m::SurfaceKind::Gravel: return "gravel";
        case g2m::SurfaceKind::Compacted: return "compacted";
        case g2m::SurfaceKind::Dirt: return "dirt";
        case g2m::SurfaceKind::Grass: return "grass";
        case g2m::SurfaceKind::Sand: return "sand";
        case g2m::SurfaceKind::Water: return "low_mu";
        case g2m::SurfaceKind::Unknown: break;
    }
    return "asphalt";
}

// racing_game core/src/session.cpp resolve_grip_surface, unchanged.
ps::SurfaceId game_resolve_grip_surface(const ps::io::SurfaceTable& table, const char* field, const std::string& name) {
    const ps::SurfaceId id = table.id_for(name);
    if (id == ps::kInvalidSurfaceId) {
        throw std::invalid_argument(std::string("Session: physics.road_surfaces.") + field + " '" + name +
                                    "' is not in the surface table");
    }
    if (id >= 255) {
        throw std::invalid_argument(std::string("Session: physics.road_surfaces.") + field + " '" + name +
                                    "' has surface id " + std::to_string(id) + " (must be < 255)");
    }
    return id;
}

// Session::setup_terrain's class-mode block, unchanged.
g2m::ps_bridge::ClassSurfaceConfig game_class_config(const ps::io::SurfaceTable& surface_table, const std::string& p,
                                                     const std::string& u, const std::string& o) {
    const ps::SurfaceId paved = game_resolve_grip_surface(surface_table, "paved", p);
    const ps::SurfaceId unpaved = game_resolve_grip_surface(surface_table, "unpaved", u);
    const ps::SurfaceId off_road = game_resolve_grip_surface(surface_table, "off_road", o);
    const auto classes = g2m::classes::SurfaceIdTable::build(
        [&surface_table, paved](g2m::SurfaceKind kind) -> std::uint16_t {
            if (kind == g2m::SurfaceKind::Unknown) return static_cast<std::uint16_t>(paved);
            const std::string name(game_surface_name_for_kind(kind));
            const ps::SurfaceId id = surface_table.id_for(name);
            if (id == ps::kInvalidSurfaceId) {
                throw std::invalid_argument("Session: terrain.class surface kind '" + name +
                                            "' is not in the surface table");
            }
            return static_cast<std::uint16_t>(id);
        },
        [paved, unpaved, off_road](g2m::LandClass land) -> std::uint16_t {
            if (land == g2m::LandClass::PavedRoad) return static_cast<std::uint16_t>(paved);
            if (land == g2m::LandClass::UnpavedRoad) return static_cast<std::uint16_t>(unpaved);
            return static_cast<std::uint16_t>(off_road);
        });
    return g2m::ps_bridge::ClassSurfaceConfig{classes, static_cast<ps::SurfaceId>(off_road)};
}

// Session::setup_terrain's road-mode block, unchanged.
g2m::ps_bridge::RoadSurfaceConfig game_road_config(const ps::io::SurfaceTable& surface_table, const std::string& p,
                                                   const std::string& u, const std::string& o) {
    const ps::SurfaceId paved = game_resolve_grip_surface(surface_table, "paved", p);
    const ps::SurfaceId unpaved = game_resolve_grip_surface(surface_table, "unpaved", u);
    const ps::SurfaceId off_road = game_resolve_grip_surface(surface_table, "off_road", o);
    const g2m::RoadValueLut lut = g2m::RoadValueLut::by_land_class(
        off_road, {{g2m::LandClass::PavedRoad, paved}, {g2m::LandClass::UnpavedRoad, unpaved}});
    return g2m::ps_bridge::RoadSurfaceConfig{lut, off_road};
}

SurfaceNames names_of(const std::string& p, const std::string& u, const std::string& o) {
    SurfaceNames n;
    n.paved = p;
    n.unpaved = u;
    n.off_road = o;
    n.kind_name = game_surface_name_for_kind;
    return n;
}

const ps::io::SurfaceTable& game_table() {
    static const ps::io::SurfaceTable table(std::string(G2M_BRIDGE_FIXTURE_DIR) + "/game_surfaces.json");
    return table;
}

struct NamesCase {
    const char *paved, *unpaved, *off_road;
};
// The game's defaults first, then other legal choices (a different name for every role).
constexpr NamesCase kNameCases[] = {{"asphalt", "dirt", "grass"},
                                    {"concrete", "gravel", "sand"},
                                    {"cobblestone", "compacted", "low_mu"},
                                    {"kerb", "grass", "dirt"}};

} // namespace

TEST_CASE("ps_bridge surfaces: the class table equals the game's, entry by entry", "[ps_bridge][surfaces]") {
    const auto& table = game_table();
    for (const auto& c : kNameCases) {
        const auto want = game_class_config(table, c.paved, c.unpaved, c.off_road);
        const auto got = bind_class_surfaces(table, names_of(c.paved, c.unpaved, c.off_road));
        REQUIRE(got.ok());
        CHECK(got->fallback == want.fallback);
        for (std::size_t i = 0; i < g2m::classes::kSurfaceKindCount; ++i) {
            INFO("surface kind " << i << " for " << c.paved << "/" << c.unpaved << "/" << c.off_road);
            CHECK(got->table.by_surface_kind[i] == want.table.by_surface_kind[i]);
        }
        for (std::size_t i = 0; i < g2m::classes::kLandClassCount; ++i) {
            INFO("land class " << i << " for " << c.paved << "/" << c.unpaved << "/" << c.off_road);
            CHECK(got->table.by_land_class[i] == want.table.by_land_class[i]);
        }
    }
    // Literal anchors so the reference cannot drift with the bridge: ids are alphabetical, asphalt first.
    const auto got = bind_class_surfaces(table, names_of("asphalt", "dirt", "grass"));
    REQUIRE(got.ok());
    CHECK(got->table.for_surface_kind(g2m::SurfaceKind::Unknown) == table.id_for("asphalt")); // Unknown -> paved
    CHECK(got->table.for_surface_kind(g2m::SurfaceKind::Water) == table.id_for("low_mu"));
    CHECK(got->table.for_surface_kind(g2m::SurfaceKind::Sett) == table.id_for("sett"));
    CHECK(got->table.for_land_class(g2m::LandClass::UnpavedRoad) == table.id_for("dirt"));
    CHECK(got->fallback == table.id_for("grass"));
}

TEST_CASE("ps_bridge surfaces: the road table equals the game's, entry by entry", "[ps_bridge][surfaces]") {
    const auto& table = game_table();
    for (const auto& c : kNameCases) {
        const auto want = game_road_config(table, c.paved, c.unpaved, c.off_road);
        const auto got = bind_road_surfaces(table, names_of(c.paved, c.unpaved, c.off_road));
        REQUIRE(got.ok());
        CHECK(got->off_road == want.off_road);
        CHECK(got->lut.v == want.lut.v); // all 16 x 16 (land class, surface) slots
    }
    const auto got = bind_road_surfaces(table, names_of("asphalt", "dirt", "grass"));
    REQUIRE(got.ok());
    CHECK(got->lut.at(g2m::LandClass::PavedRoad, g2m::SurfaceKind::Cobblestone) == table.id_for("asphalt"));
    CHECK(got->lut.at(g2m::LandClass::UnpavedRoad, g2m::SurfaceKind::Unknown) == table.id_for("dirt"));
    CHECK(got->lut.at(g2m::LandClass::TreeCover, g2m::SurfaceKind::Asphalt) == table.id_for("grass"));
}

TEST_CASE("ps_bridge surfaces: a name missing from the table is an error with the game's text", "[ps_bridge][surfaces]") {
    const auto& table = game_table();
    {
        const auto r = bind_road_surfaces(table, names_of("asphalt", "no_such_surface", "grass"));
        REQUIRE(!r.ok());
        CHECK(r.error().code == g2m::ErrorCode::InvalidArgument);
        CHECK(r.error().message == "Session: physics.road_surfaces.unpaved 'no_such_surface' is not in the surface table");
    }
    {
        // paved is checked first, then unpaved, then off_road.
        const auto r = bind_class_surfaces(table, names_of("nope_a", "nope_b", "nope_c"));
        REQUIRE(!r.ok());
        CHECK(r.error().message == "Session: physics.road_surfaces.paved 'nope_a' is not in the surface table");
        const auto r2 = bind_class_surfaces(table, names_of("asphalt", "dirt", "nope_c"));
        REQUIRE(!r2.ok());
        CHECK(r2.error().message == "Session: physics.road_surfaces.off_road 'nope_c' is not in the surface table");
    }
    {
        // A surface kind whose name is not in the table: the first such kind (SurfaceKind order) is reported.
        SurfaceNames n = names_of("asphalt", "dirt", "grass");
        n.kind_name = [](g2m::SurfaceKind k) -> std::string {
            return k == g2m::SurfaceKind::Concrete || k == g2m::SurfaceKind::Sand ? "missing_kind" : game_surface_name_for_kind(k);
        };
        const auto r = bind_class_surfaces(table, n);
        REQUIRE(!r.ok());
        CHECK(r.error().message == "Session: terrain.class surface kind 'missing_kind' is not in the surface table");
        // Road mode never asks for kind names.
        CHECK(bind_road_surfaces(table, n).ok());
    }
}

TEST_CASE("ps_bridge surfaces: an id of 255 or more is refused (the 8-bit Jolt material index)", "[ps_bridge][surfaces]") {
    // 300 entries s000..s299: ids are alphabetical, so s254 is id 254 and s255 is id 255.
    const auto path = std::filesystem::temp_directory_path() / "g2m_ps_bridge_surfaces_300.json";
    {
        std::ofstream out(path, std::ios::binary);
        out << "{\"format\": \"physics_sim.surfaces/1\", \"surfaces\": {";
        for (int i = 0; i < 300; ++i) {
            char name[8];
            std::snprintf(name, sizeof name, "s%03d", i);
            out << (i ? "," : "") << "\"" << name << "\": {\"lambda_mu\": 1.0, \"crr\": 0.01}";
        }
        out << "}}";
    }
    const ps::io::SurfaceTable table(path.string());
    REQUIRE(table.id_for("s254") == 254);
    REQUIRE(table.id_for("s255") == 255);
    CHECK(bind_road_surfaces(table, names_of("s000", "s253", "s254")).ok()); // 254 is the last legal id
    const auto r = bind_road_surfaces(table, names_of("s000", "s255", "s254"));
    REQUIRE(!r.ok());
    CHECK(r.error().code == g2m::ErrorCode::InvalidArgument);
    CHECK(r.error().message == "Session: physics.road_surfaces.unpaved 's255' has surface id 255 (must be < 255)");
    const auto r2 = bind_class_surfaces(table, names_of("s299", "s000", "s000"));
    REQUIRE(!r2.ok());
    CHECK(r2.error().message == "Session: physics.road_surfaces.paved 's299' has surface id 299 (must be < 255)");
    std::error_code ec;
    std::filesystem::remove(path, ec);
}
