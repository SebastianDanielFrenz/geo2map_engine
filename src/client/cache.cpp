// src/client/cache.cpp - see g2m/client/cache.h.

#include "g2m/client/cache.h"

#include "g2m/layer/release.h"

#include <string>

namespace g2m {
namespace {

TileAddress address(const ReleaseId& rid, std::string_view layer, const TileKey& key) {
    return TileAddress{to_hex(rid), std::string(layer), key};
}

} // namespace

Result<std::unique_ptr<ClientCache>> ClientCache::open(const ClientCacheConfig& config) {
    TileStoreConfig sc;
    sc.dir = config.dir;
    sc.name = "cache";
    sc.size_cap_bytes = config.size_cap_bytes;
    sc.inline_limit = config.inline_limit;
    sc.busy_timeout_ms = config.busy_timeout_ms;
    auto store = TileStore::open(sc);
    if (!store) return store.error();
    return std::unique_ptr<ClientCache>(new ClientCache(std::move(*store)));
}

Result<void> ClientCache::put(std::span<const std::uint8_t> container, bool pin) {
    auto view = parse_container(container);
    if (!view) return view.error();
    if (view->header.rid == ReleaseId{}) {
        return make_error(ErrorCode::InvalidArgument, "client cache: container has no release id");
    }
    return store_->put(address(view->header.rid, view->header.layer, view->header.key), view->header, view->body,
                       pin);
}

Result<std::optional<std::vector<std::uint8_t>>> ClientCache::get(const ReleaseId& rid, std::string_view layer,
                                                                  const TileKey& key) {
    auto t = store_->get(address(rid, layer, key));
    if (!t) return t.error();
    if (!*t) return std::optional<std::vector<std::uint8_t>>{};
    return std::optional<std::vector<std::uint8_t>>{(*t)->container()};
}

Result<bool> ClientCache::set_pinned(const ReleaseId& rid, std::string_view layer, const TileKey& key, bool pinned) {
    return store_->set_pinned(address(rid, layer, key), pinned);
}

Result<std::uint64_t> ClientCache::set_pinned_range(const ReleaseId& rid, std::string_view layer, const TileKey& lo,
                                                    const TileKey& hi, bool pinned) {
    return store_->set_pinned_range(to_hex(rid), std::string(layer), lo, hi, pinned);
}

Result<bool> ClientCache::remove(const ReleaseId& rid, std::string_view layer, const TileKey& key) {
    return store_->remove(address(rid, layer, key));
}

Result<TileStoreStats> ClientCache::stats() { return store_->stats(); }

Result<void> ClientCache::check_consistency() { return store_->check_consistency(); }

} // namespace g2m
