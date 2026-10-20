#include "g2m/ps_bridge/deck_installer.h"

#include "ps/world/world.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace g2m::ps_bridge {

namespace {
using Clock = std::chrono::steady_clock;

std::uint64_t ns_since(Clock::time_point t0) {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count());
}

void atomic_max(std::atomic<std::uint64_t>& a, std::uint64_t v) {
    std::uint64_t cur = a.load(std::memory_order_relaxed);
    while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
    }
}
} // namespace

DeckKey deck_key(const RoadDeck& deck) {
    return DeckKey{deck.way_id, static_cast<int>(std::round(deck.start_station)), static_cast<int>(std::round(deck.end_station))};
}

ps::MeshShape make_deck_mesh_shape(const RoadDeck& deck, ps::SurfaceId surface) {
    const auto& mesh = deck.mesh;
    ps::MeshShape shape;
    shape.vertices.reserve(mesh.positions.size() / 3);
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        shape.vertices.push_back({mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]});
    }
    shape.indices.assign(mesh.indices.begin(), mesh.indices.end());
    shape.triangle_surface_ids.assign(shape.indices.size() / 3, surface);
    return shape;
}

DeckInstaller::DeckInstaller(const ps::World& world, Options options) : world_(world), options_(std::move(options)) {
    const unsigned n = std::max(1u, options_.workers);
    workers_.reserve(n);
    for (unsigned i = 0; i < n; ++i) workers_.emplace_back([this] { worker_main(); });
}

DeckInstaller::~DeckInstaller() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& t : workers_) {
        if (t.joinable()) t.join();
    }
    // entries_ (and the handles still in it) die here, after the workers are gone.
}

void DeckInstaller::worker_main() {
    for (;;) {
        DeckKey key;
        std::shared_ptr<const RoadDeck> deck;
        ps::SurfaceId surface = ps::kInvalidSurfaceId;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
            if (stop_) return;
            key = queue_.front();
            queue_.pop_front();
            Entry& e = entries_.at(key);
            e.state = State::Building;
            deck = e.deck;
            surface = e.surface;
        }
        if (options_.before_build) options_.before_build(key);
        ps::ShapeHandle handle;
        bool ok = true;
        const auto t0 = Clock::now();
        try {
            handle = world_.create_shape(ps::ShapeDesc{make_deck_mesh_shape(*deck, surface)});
        } catch (...) { // TOOL-019: catch by type only; the message is not read
            ok = false;
        }
        const std::uint64_t ns = ns_since(t0);
        build_ns_total_.fetch_add(ns, std::memory_order_relaxed);
        atomic_max(build_ns_max_, ns);
        if (ok) {
            built_.fetch_add(1, std::memory_order_relaxed);
        } else {
            failed_.fetch_add(1, std::memory_order_relaxed);
            std::fprintf(stderr, "RG_DECK build_failed way=%lld start=%d end=%d\n", static_cast<long long>(key.way_id),
                         key.start, key.end);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Entry& e = entries_.at(key);
            e.handle = std::move(handle);
            e.state = ok ? State::Ready : State::Failed;
            e.deck.reset(); // the mesh is no longer needed once the shape exists
        }
    }
}

void DeckInstaller::request(const DeckCandidate& c) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto [it, inserted] = entries_.try_emplace(c.key);
    if (!inserted) return;
    it->second.deck = c.deck;
    it->second.surface = c.surface;
    queue_.push_back(c.key);
    requested_.fetch_add(1, std::memory_order_relaxed);
    cv_.notify_one();
}

bool DeckInstaller::installed(const DeckKey& key) const {
    // Sim thread only; entries_ is mutated under mutex_ by workers (state only), so take the lock.
    auto& self = const_cast<DeckInstaller&>(*this);
    std::lock_guard<std::mutex> lock(self.mutex_);
    const auto it = entries_.find(key);
    return it != entries_.end() && it->second.state == State::Installed;
}

bool DeckInstaller::update(ps::World& world, std::span<const DeckCandidate> required,
                           std::span<const DeckCandidate> prefetch, int budget) {
    for (const auto& c : required) request(c);
    for (const auto& c : prefetch) request(c);

    int installed_now = 0;
    for (const auto& c : required) {
        ps::ShapeHandle handle;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Entry& e = entries_.at(c.key);
            if (e.state == State::Installed || e.state == State::Failed) continue;
            if (e.state != State::Ready) { // Queued or Building: strict order, hold every later deck back
                ++waits_;
                return false;
            }
            if (budget > 0 && installed_now >= budget) return false; // more to install next call
            handle = e.handle;
        }
        ps::BodyDesc body;
        body.motion = ps::BodyMotionType::Static;
        body.pose.position = {c.x, c.y, 0.0};
        const auto t0 = Clock::now();
        const ps::BodyId id = world.create_body(body, handle);
        const std::uint64_t ns = ns_since(t0);
        install_ns_total_ += ns;
        install_ns_max_ = std::max(install_ns_max_, ns);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Entry& e = entries_.at(c.key);
            e.state = State::Installed;
            e.handle.reset(); // the body keeps the shape alive
        }
        handle.reset();
        bodies_.push_back(id);
        ++installed_;
        ++installed_now;
    }
    return true; // every required deck is installed (or failed); installing is O(1), so no extra frozen tick
}

bool DeckInstaller::update_legacy(ps::World& world, std::span<const DeckCandidate> required, int budget) {
    int installed_now = 0;
    for (const auto& c : required) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = entries_.find(c.key);
            if (it != entries_.end() && it->second.state == State::Installed) continue;
        }
        if (budget > 0 && installed_now >= budget) return false;
        ps::BodyDesc body;
        body.motion = ps::BodyMotionType::Static;
        body.pose.position = {c.x, c.y, 0.0};
        const auto t0 = Clock::now();
        body.shape = make_deck_mesh_shape(*c.deck, c.surface);
        const ps::BodyId id = world.create_body(body);
        const std::uint64_t ns = ns_since(t0);
        install_ns_total_ += ns;
        install_ns_max_ = std::max(install_ns_max_, ns);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            entries_[c.key].state = State::Installed;
        }
        bodies_.push_back(id);
        ++installed_;
        ++installed_now;
    }
    return budget <= 0 || installed_now == 0;
}

std::size_t DeckInstaller::wait_until_installed(ps::World& world, std::span<const DeckCandidate> required) {
    const std::uint64_t before = installed_;
    const auto t0 = Clock::now();
    while (!update(world, required, {}, 0)) {
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    wait_ns_total_ += ns_since(t0);
    return static_cast<std::size_t>(installed_ - before);
}

DeckInstallStats DeckInstaller::stats() const {
    DeckInstallStats s;
    s.requested = requested_.load(std::memory_order_relaxed);
    s.built = built_.load(std::memory_order_relaxed);
    s.failed = failed_.load(std::memory_order_relaxed);
    s.installed = installed_;
    s.waits = waits_;
    s.install_ns_total = install_ns_total_;
    s.install_ns_max = install_ns_max_;
    s.build_ns_total = build_ns_total_.load(std::memory_order_relaxed);
    s.build_ns_max = build_ns_max_.load(std::memory_order_relaxed);
    s.wait_ns_total = wait_ns_total_;
    return s;
}

DeckSelection select_decks(std::span<const std::shared_ptr<const RoadDeck>> decks, double e, double n, double e0_m,
                           double n0_m, double required_radius_m, double prefetch_radius_m, ps::SurfaceId paved,
                           ps::SurfaceId unpaved, bool legacy_radius, const DeckInstaller* skip_installed) {
    DeckSelection out;
    const double radius = required_radius_m;
    const double reach = legacy_radius ? radius : prefetch_radius_m;
    for (const auto& deck : decks) {
        const DeckKey key = deck_key(*deck);
        if (skip_installed != nullptr && skip_installed->installed(key)) continue;
        const double dx = std::max({deck->min_easting - e, e - deck->max_easting, 0.0});
        const double dy = std::max({deck->min_northing - n, n - deck->max_northing, 0.0});
        const double d2 = dx * dx + dy * dy;
        if (d2 > reach * reach) continue;
        DeckCandidate c;
        c.key = key;
        c.deck = deck;
        c.surface = deck->land_class == g2m::LandClass::PavedRoad ? paved : unpaved;
        c.x = deck->mesh.origin[0] - e0_m;
        c.y = deck->mesh.origin[1] - n0_m;
        (d2 <= radius * radius ? out.required : out.prefetch).push_back(std::move(c));
    }
    const auto by_key = [](const DeckCandidate& a, const DeckCandidate& b) { return a.key < b.key; };
    std::stable_sort(out.required.begin(), out.required.end(), by_key);
    std::stable_sort(out.prefetch.begin(), out.prefetch.end(), by_key);
    return out;
}

} // namespace g2m::ps_bridge
