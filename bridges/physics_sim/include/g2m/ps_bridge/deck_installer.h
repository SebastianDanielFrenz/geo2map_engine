// g2m/ps_bridge/deck_installer.h - road deck collision bodies, built off the simulation thread (S1, design 5.2 option B).
//
// physics_sim R2 (d1f40ad): ps::World::create_shape(ShapeDesc) is const and callable from ANY thread, also while a
// step runs, and returns a ref-counted ShapeHandle; ps::World::create_body(desc, handle) on the sim thread between
// steps is O(1) in the shape size (48k-triangle deck: 6.4 us vs 22.4 ms) and bit-identical to create_body(desc).
//
// DeckInstaller owns a few plain std::threads (never ps::jobs workers: create_shape allocates, which a job would
// count as a debug_alloc violation) that convert a RoadDeck's mesh to a ps::MeshShape and build its handle. The sim
// thread calls update() between ticks:
//
//   * every REQUIRED deck is queued for building (and every PREFETCH deck: decks soon to be required, so the build
//     is done by the time the car gets there);
//   * required decks are installed in the FIXED order the caller passes them in ((way_id, start, end)), never in
//     builder-completion order - BodyIds, and with them the Jolt solver order and World::state_hash, follow that
//     order only. The order is strict: a required deck whose shape is not ready yet holds back every later one, and
//     update() returns false (the Session freezes the clock) until all required decks are installed;
//   * at most `budget` decks are installed per call.
//
// A deck whose mesh Jolt rejects fails on its worker (stderr once, counted) and is treated as done without a body.
//
// Threading: update()/bodies()/stats() on the sim (stepping) thread only. The installer must be destroyed before the
// World it was given (the workers call World::create_shape); the handles it hands to bodies may outlive both.
#pragma once

#include "g2m/world/road_structures.h"

#include "ps/backend/shape_desc.h"
#include "ps/backend/shape_handle.h"
#include "ps/types.h"
#include "ps/world/ids.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

namespace ps {
class World;
}

namespace g2m::ps_bridge {

using g2m::world::RoadDeck;

// The identity of a deck, ordered the way decks are installed.
struct DeckKey {
    std::int64_t way_id = 0;
    int start = 0; // rounded stations, metres
    int end = 0;
    friend bool operator==(const DeckKey&, const DeckKey&) = default;
    friend bool operator<(const DeckKey& a, const DeckKey& b) {
        if (a.way_id != b.way_id) return a.way_id < b.way_id;
        if (a.start != b.start) return a.start < b.start;
        return a.end < b.end;
    }
};

[[nodiscard]] DeckKey deck_key(const RoadDeck& deck);

struct DeckCandidate {
    DeckKey key;
    std::shared_ptr<const RoadDeck> deck;
    ps::SurfaceId surface = ps::kInvalidSurfaceId;
    // Static body pose in session coordinates (the deck mesh is relative to its own origin).
    double x = 0.0, y = 0.0;
};

// The collision mesh of a deck: its vertices/indices with one surface id on every triangle.
[[nodiscard]] ps::MeshShape make_deck_mesh_shape(const RoadDeck& deck, ps::SurfaceId surface);

struct DeckInstallStats {
    std::uint64_t requested = 0;       // decks queued for building
    std::uint64_t built = 0;           // shapes built by the workers
    std::uint64_t failed = 0;          // shapes the backend rejected
    std::uint64_t installed = 0;       // bodies created
    std::uint64_t waits = 0;           // update() calls that held back for an unready shape
    std::uint64_t install_ns_total = 0; // sim-thread time inside create_body(desc, handle), all installs
    std::uint64_t install_ns_max = 0;   // worst single install
    std::uint64_t build_ns_total = 0;   // worker time inside create_shape (off the sim thread)
    std::uint64_t build_ns_max = 0;
    std::uint64_t wait_ns_total = 0;    // sim-thread time spent blocking in wait_until_installed()
};

class DeckInstaller {
public:
    struct Options {
        unsigned workers = 2;
        // Test hook: called on the worker right before a deck's shape is built (the determinism test sleeps a
        // different amount per deck to shuffle the completion order).
        std::function<void(const DeckKey&)> before_build;
    };

    DeckInstaller(const ps::World& world, Options options);
    ~DeckInstaller();
    DeckInstaller(const DeckInstaller&) = delete;
    DeckInstaller& operator=(const DeckInstaller&) = delete;

    // Both lists in ascending DeckKey order. Returns true when every required deck is installed (or failed) after this
    // call - including decks it installed itself: an install is O(1), so the clock need not hold another tick for
    // it (the reference path update_legacy() still does, as the per-deck path always did). False while a required
    // shape is still building or the budget ran out. budget <= 0: install without a limit.
    [[nodiscard]] bool update(ps::World& world, std::span<const DeckCandidate> required,
                              std::span<const DeckCandidate> prefetch, int budget);

    // Reference path (the pre-S1 behaviour, kept as the plain A/B reference per the owner's "keep a reference for
    // optimised code" rule): builds each required deck's shape ON THE CALLING (sim) THREAD inside create_body(desc),
    // in key order, `budget` decks per call. Same return convention as update(). Used by SessionConfig::
    // legacy_deck_install and by the determinism test; its stats (install_ns_*) are the "before" numbers.
    [[nodiscard]] bool update_legacy(ps::World& world, std::span<const DeckCandidate> required, int budget);

    // Start-up path: update() with no budget until the required decks are all installed, sleeping while workers
    // build. Returns the number of decks installed.
    std::size_t wait_until_installed(ps::World& world, std::span<const DeckCandidate> required);

    [[nodiscard]] const std::vector<ps::BodyId>& bodies() const { return bodies_; } // installed, in install order
    [[nodiscard]] bool installed(const DeckKey& key) const;
    [[nodiscard]] DeckInstallStats stats() const;

private:
    enum class State : std::uint8_t { Queued, Building, Ready, Failed, Installed };
    struct Entry {
        State state = State::Queued;
        std::shared_ptr<const RoadDeck> deck;
        ps::SurfaceId surface = ps::kInvalidSurfaceId;
        ps::ShapeHandle handle;
    };

    void request(const DeckCandidate& c);
    void worker_main();

    const ps::World& world_;
    Options options_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::map<DeckKey, Entry> entries_;
    std::deque<DeckKey> queue_;
    bool stop_ = false;
    std::vector<std::thread> workers_;

    std::vector<ps::BodyId> bodies_; // sim thread only

    std::atomic<std::uint64_t> requested_{0}, built_{0}, failed_{0}, build_ns_total_{0}, build_ns_max_{0};
    std::uint64_t installed_ = 0, waits_ = 0, install_ns_total_ = 0, install_ns_max_ = 0, wait_ns_total_ = 0; // sim thread
};

// The deck selection loop of the game's Session::sync_road_decks (moved as it is): which of `decks` the installer
// must have now (REQUIRED: within required_radius_m of the focus) and which it should only build ahead (PREFETCH:
// within prefetch_radius_m, or only required_radius_m when legacy_radius), each list in ascending DeckKey order
// (stable sort, so two decks with one key keep their input order).
//   * (e, n) is the focus in absolute UTM metres, (e0_m, n0_m) the session frame origin: a candidate's body pose is
//     its mesh origin minus that origin (the game's frame().e0_m()/n0_m()).
//   * The distance is the one to the deck's UTM bounding box (0 inside it).
//   * paved is used for a deck of LandClass::PavedRoad, unpaved for every other class.
//   * skip_installed (optional): a deck the installer already installed is left out, as the game does. Omitting it
//     only adds already-installed decks to `required`, which update() skips anyway.
struct DeckSelection {
    std::vector<DeckCandidate> required;
    std::vector<DeckCandidate> prefetch;
};
[[nodiscard]] DeckSelection select_decks(std::span<const std::shared_ptr<const RoadDeck>> decks, double e, double n,
                                         double e0_m, double n0_m, double required_radius_m, double prefetch_radius_m,
                                         ps::SurfaceId paved, ps::SurfaceId unpaved, bool legacy_radius,
                                         const DeckInstaller* skip_installed = nullptr);

} // namespace g2m::ps_bridge
