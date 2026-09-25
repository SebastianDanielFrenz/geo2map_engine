// src/server/tile_store.cpp - see g2m/server/tile_store.h and
// docs/formats/cache_db.md.

#include "g2m/server/tile_store.h"

#include "g2m/layer/layer_id.h"
#include "g2m/layer/release.h"

#include <sqlite3.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <system_error>
#include <thread>

namespace g2m {
namespace {

constexpr int kSchemaVersion = 1;
constexpr std::size_t kMaxScopeLength = 128;

const char* const kSchema = R"sql(
CREATE TABLE meta(name TEXT PRIMARY KEY, value INTEGER NOT NULL) WITHOUT ROWID;
INSERT INTO meta(name, value) VALUES('body_bytes', 0);
CREATE TABLE bodies(
  id INTEGER PRIMARY KEY,
  hash BLOB NOT NULL UNIQUE,
  size INTEGER NOT NULL,
  refs INTEGER NOT NULL,
  data BLOB);
CREATE TABLE tiles(
  scope TEXT NOT NULL,
  layer TEXT NOT NULL,
  key INTEGER NOT NULL,
  hash BLOB NOT NULL,
  header BLOB NOT NULL,
  last_used INTEGER NOT NULL,
  pinned INTEGER NOT NULL,
  PRIMARY KEY(scope, layer, key)) WITHOUT ROWID;
CREATE INDEX tiles_lru ON tiles(pinned, last_used);
CREATE INDEX tiles_used ON tiles(last_used);
CREATE INDEX tiles_hash ON tiles(hash);
PRAGMA user_version = 1;
)sql";

bool valid_store_name(const std::string& s) {
    if (s.empty() || s.size() > 64) return false;
    for (char c : s) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    }
    return true;
}

Result<void> check_address(const TileAddress& a) {
    if (a.scope.empty() || a.scope.size() > kMaxScopeLength) {
        return make_error(ErrorCode::InvalidArgument, "tile store: scope must be 1..128 bytes");
    }
    if (!is_valid_layer_name(a.layer)) return make_error(ErrorCode::InvalidArgument, "tile store: invalid layer name");
    if (!a.key.valid()) return make_error(ErrorCode::InvalidArgument, "tile store: invalid key");
    return {};
}

std::string path_utf8(const std::filesystem::path& p) {
    const std::u8string u = p.u8string();
    return std::string(u.begin(), u.end());
}

ContentHash hash_from_blob(const void* p, int n) {
    ContentHash h{};
    if (p && n == 32) std::memcpy(h.data(), p, 32);
    return h;
}

} // namespace

std::vector<std::uint8_t> StoredTile::container() const {
    std::vector<std::uint8_t> c;
    c.reserve(header_bytes.size() + body.size());
    c.insert(c.end(), header_bytes.begin(), header_bytes.end());
    c.insert(c.end(), body.begin(), body.end());
    return c;
}

// ---------------------------------------------------------------------------

struct TileStore::Impl {
    TileStoreConfig cfg;
    std::filesystem::path blob_dir;
    sqlite3* db = nullptr;
    std::mutex mu; // guards db and every cached statement
    std::map<const char*, sqlite3_stmt*> stmts;
    std::atomic<std::uint64_t> integrity_drops{0};
    std::atomic<std::uint64_t> evicted{0};

    ~Impl() {
        for (auto& [sql, s] : stmts) sqlite3_finalize(s);
        if (db) sqlite3_close(db);
    }

    Error db_error(int rc, const char* what) const {
        const ErrorCode code = (rc == SQLITE_BUSY || rc == SQLITE_LOCKED) ? ErrorCode::Busy : ErrorCode::Io;
        return make_error(code, std::string("tile store: ") + what + ": " + (db ? sqlite3_errmsg(db) : sqlite3_errstr(rc)));
    }

    Result<void> exec(const char* sql) {
        const int rc = sqlite3_exec(db, sql, nullptr, nullptr, nullptr);
        if (rc != SQLITE_OK) return db_error(rc, sql);
        return {};
    }

    // A cached prepared statement, reset and unbound when the Query dies.
    class Query {
    public:
        Query(Impl& impl, const char* sql) {
            auto it = impl.stmts.find(sql);
            if (it == impl.stmts.end()) {
                sqlite3_stmt* s = nullptr;
                rc_ = sqlite3_prepare_v3(impl.db, sql, -1, SQLITE_PREPARE_PERSISTENT, &s, nullptr);
                if (rc_ != SQLITE_OK) return;
                it = impl.stmts.emplace(sql, s).first;
            }
            s_ = it->second;
        }
        ~Query() {
            if (s_) {
                sqlite3_reset(s_);
                sqlite3_clear_bindings(s_);
            }
        }
        Query(const Query&) = delete;
        Query& operator=(const Query&) = delete;

        bool ok() const { return s_ != nullptr; }
        Query& text(int i, const std::string& v) {
            if (s_) sqlite3_bind_text(s_, i, v.data(), static_cast<int>(v.size()), SQLITE_TRANSIENT);
            return *this;
        }
        Query& blob(int i, std::span<const std::uint8_t> v) {
            if (s_ && v.empty()) {
                sqlite3_bind_zeroblob(s_, i, 0); // a NULL data pointer would bind SQL NULL
            } else if (s_) {
                sqlite3_bind_blob64(s_, i, v.data(), v.size(), SQLITE_TRANSIENT);
            }
            return *this;
        }
        Query& null(int i) {
            if (s_) sqlite3_bind_null(s_, i);
            return *this;
        }
        Query& i64(int i, std::int64_t v) {
            if (s_) sqlite3_bind_int64(s_, i, v);
            return *this;
        }
        Query& addr(const TileAddress& a) { // ?1 scope, ?2 layer, ?3 key
            text(1, a.scope);
            text(2, a.layer);
            return i64(3, static_cast<std::int64_t>(a.key.packed()));
        }
        // SQLITE_ROW, SQLITE_DONE or an error code.
        int step() {
            if (!s_) return rc_ == SQLITE_OK ? SQLITE_ERROR : rc_;
            return sqlite3_step(s_);
        }
        std::int64_t col_i64(int c) const { return sqlite3_column_int64(s_, c); }
        std::string col_text(int c) const {
            const auto* p = reinterpret_cast<const char*>(sqlite3_column_text(s_, c));
            return p ? std::string(p, static_cast<std::size_t>(sqlite3_column_bytes(s_, c))) : std::string();
        }
        std::vector<std::uint8_t> col_blob(int c) const {
            const auto* p = static_cast<const std::uint8_t*>(sqlite3_column_blob(s_, c));
            const int n = sqlite3_column_bytes(s_, c);
            return p ? std::vector<std::uint8_t>(p, p + n) : std::vector<std::uint8_t>();
        }
        ContentHash col_hash(int c) const { return hash_from_blob(sqlite3_column_blob(s_, c), sqlite3_column_bytes(s_, c)); }

    private:
        sqlite3_stmt* s_ = nullptr;
        int rc_ = SQLITE_OK;
    };

    // Runs a statement that returns no rows (or ignores them).
    Result<void> run(Query& q, const char* what) {
        int rc = q.step();
        while (rc == SQLITE_ROW) rc = q.step();
        if (rc != SQLITE_DONE) return db_error(rc, what);
        return {};
    }

    // BEGIN IMMEDIATE ... COMMIT, rolled back unless committed.
    class Txn {
    public:
        explicit Txn(Impl& impl) : impl_(impl) {}
        ~Txn() {
            if (open_) (void)impl_.exec("ROLLBACK");
        }
        Result<void> begin() {
            if (auto r = impl_.exec("BEGIN IMMEDIATE"); !r) return r;
            open_ = true;
            return {};
        }
        Result<void> commit() {
            auto r = impl_.exec("COMMIT");
            if (r) open_ = false;
            return r;
        }

    private:
        Impl& impl_;
        bool open_ = false;
    };

    std::filesystem::path blob_path(const ContentHash& h) const {
        const std::string hex = to_hex(h);
        return blob_dir / hex.substr(0, 2) / hex.substr(2, 2) / hex;
    }

    Result<void> write_blob(const ContentHash& h, std::span<const std::uint8_t> bytes) const {
        const std::filesystem::path p = blob_path(h);
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
        if (ec) return make_error(ErrorCode::Io, "tile store: cannot create " + path_utf8(p.parent_path()));
        std::filesystem::path tmp = p;
        tmp += ".tmp";
        {
            std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
            f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            f.flush();
            if (!f) return make_error(ErrorCode::Io, "tile store: cannot write " + path_utf8(tmp));
        }
        // Windows refuses to replace (or delete) a file another reader still
        // has open, which leaves an orphan behind a dropped body. Blob files are
        // content-addressed: an existing file with exactly these bytes is already
        // the right file. Anything else is retried briefly (readers hold a file
        // only while copying it), then reported.
        for (int attempt = 0;; ++attempt) {
            std::filesystem::rename(tmp, p, ec);
            if (!ec) return {};
            if (const auto existing = read_blob(h, bytes.size()); existing && content_hash(*existing) == h) {
                std::filesystem::remove(tmp, ec);
                return {};
            }
            if (attempt == 50) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::filesystem::remove(tmp, ec);
        return make_error(ErrorCode::Io, "tile store: cannot rename " + path_utf8(tmp));
    }

    // The file's bytes if it exists with exactly `size` bytes.
    std::optional<std::vector<std::uint8_t>> read_blob(const ContentHash& h, std::uint64_t size) const {
        std::error_code ec;
        const std::filesystem::path p = blob_path(h);
        const auto actual = std::filesystem::file_size(p, ec);
        if (ec || actual != size) return std::nullopt;
        std::ifstream f(p, std::ios::binary);
        std::vector<std::uint8_t> v(static_cast<std::size_t>(size));
        f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(v.size()));
        if (!f || f.gcount() != static_cast<std::streamsize>(v.size())) return std::nullopt;
        return v;
    }

    bool blob_ok_size(const ContentHash& h, std::uint64_t size) const {
        std::error_code ec;
        const auto actual = std::filesystem::file_size(blob_path(h), ec);
        return !ec && actual == size;
    }

    Result<std::int64_t> body_bytes() {
        Query q(*this, "SELECT value FROM meta WHERE name = 'body_bytes'");
        if (q.step() != SQLITE_ROW) return db_error(SQLITE_ERROR, "read body_bytes");
        return q.col_i64(0);
    }

    Result<void> add_body_bytes(std::int64_t delta) {
        Query q(*this, "UPDATE meta SET value = value + ?1 WHERE name = 'body_bytes'");
        q.i64(1, delta);
        return run(q, "update body_bytes");
    }

    // Drops one reference; deletes the body (and its file) at zero. Must
    // run inside a write transaction.
    Result<void> decref(const ContentHash& h) {
        std::int64_t refs = 0, size = 0;
        bool is_file = false;
        {
            Query q(*this, "UPDATE bodies SET refs = refs - 1 WHERE hash = ?1 RETURNING refs, size, data IS NULL");
            q.blob(1, h);
            const int rc = q.step();
            if (rc == SQLITE_DONE) return {}; // already gone
            if (rc != SQLITE_ROW) return db_error(rc, "decref");
            refs = q.col_i64(0);
            size = q.col_i64(1);
            is_file = q.col_i64(2) != 0;
            if (auto r = run(q, "decref"); !r) return r;
        }
        if (refs > 0) return {};
        {
            Query q(*this, "DELETE FROM bodies WHERE hash = ?1");
            q.blob(1, h);
            if (auto r = run(q, "delete body"); !r) return r;
        }
        if (auto r = add_body_bytes(-size); !r) return r;
        if (is_file) {
            std::error_code ec;
            std::filesystem::remove(blob_path(h), ec); // a missing file is fine
        }
        return {};
    }

    // Deletes the tile row; returns its hash if it existed.
    Result<std::optional<ContentHash>> delete_row(const TileAddress& a) {
        Query q(*this, "DELETE FROM tiles WHERE scope = ?1 AND layer = ?2 AND key = ?3 RETURNING hash");
        q.addr(a);
        const int rc = q.step();
        if (rc == SQLITE_DONE) return std::optional<ContentHash>();
        if (rc != SQLITE_ROW) return db_error(rc, "delete tile");
        const ContentHash h = q.col_hash(0);
        if (auto r = run(q, "delete tile"); !r) return r.error();
        return std::optional<ContentHash>(h);
    }

    Result<void> drop_entry(const TileAddress& a) {
        auto h = delete_row(a);
        if (!h) return h.error();
        if (*h) return decref(**h);
        return {};
    }

    // A damaged body is damaged for every address: deletes every tile row
    // referring to it, the body row and its file. Returns the rows deleted.
    // Must run inside a write transaction.
    Result<std::uint64_t> drop_body(const ContentHash& h) {
        std::uint64_t n = 0;
        {
            Query q(*this, "DELETE FROM tiles WHERE hash = ?1");
            q.blob(1, h);
            if (auto r = run(q, "drop body tiles"); !r) return r.error();
            n = static_cast<std::uint64_t>(sqlite3_changes(db));
        }
        std::int64_t size = 0;
        bool is_file = false;
        {
            Query q(*this, "DELETE FROM bodies WHERE hash = ?1 RETURNING size, data IS NULL");
            q.blob(1, h);
            const int rc = q.step();
            if (rc == SQLITE_DONE) return n;
            if (rc != SQLITE_ROW) return db_error(rc, "drop body");
            size = q.col_i64(0);
            is_file = q.col_i64(1) != 0;
            if (auto r = run(q, "drop body"); !r) return r.error();
        }
        if (auto r = add_body_bytes(-size); !r) return r.error();
        if (is_file) {
            std::error_code ec;
            std::filesystem::remove(blob_path(h), ec);
        }
        return n;
    }

    // Evicts LRU unpinned entries (never `keep`) until body_bytes <= target.
    // Must run inside a write transaction.
    Result<std::uint64_t> evict_locked(std::uint64_t target, const TileAddress* keep) {
        std::uint64_t n = 0;
        for (;;) {
            auto total = body_bytes();
            if (!total) return total.error();
            if (static_cast<std::uint64_t>(*total) <= target) return n;
            std::vector<TileAddress> victims;
            {
                Query q(*this, "SELECT scope, layer, key FROM tiles WHERE pinned = 0 ORDER BY last_used LIMIT 32");
                int rc = 0;
                while ((rc = q.step()) == SQLITE_ROW) {
                    auto key = TileKey::unpack(static_cast<std::uint64_t>(q.col_i64(2)));
                    TileAddress a{q.col_text(0), q.col_text(1), key ? *key : TileKey{}};
                    if (keep && a == *keep) continue;
                    victims.push_back(std::move(a));
                }
                if (rc != SQLITE_DONE) return db_error(rc, "select LRU");
            }
            if (victims.empty()) return n;
            for (const TileAddress& a : victims) {
                if (auto r = drop_entry(a); !r) return r.error();
                ++n;
                evicted.fetch_add(1, std::memory_order_relaxed);
                auto t = body_bytes();
                if (!t) return t.error();
                if (static_cast<std::uint64_t>(*t) <= target) return n;
            }
        }
    }

    Result<void> touch(const TileAddress& a) {
        Query q(*this, "UPDATE tiles SET last_used = (SELECT COALESCE(MAX(last_used), 0) + 1 FROM tiles) "
                       "WHERE scope = ?1 AND layer = ?2 AND key = ?3");
        q.addr(a);
        return run(q, "touch");
    }

    struct Row {
        ContentHash hash{};
        std::vector<std::uint8_t> header;
        bool pinned = false;
        std::uint64_t size = 0;
        bool is_file = false;
        std::vector<std::uint8_t> data; // inline bodies only
    };
    Result<std::optional<Row>> select_row(const TileAddress& a) {
        Query q(*this, "SELECT t.hash, t.header, t.pinned, b.size, b.data IS NULL, b.data FROM tiles t "
                       "JOIN bodies b ON b.hash = t.hash WHERE t.scope = ?1 AND t.layer = ?2 AND t.key = ?3");
        q.addr(a);
        const int rc = q.step();
        if (rc == SQLITE_DONE) return std::optional<Row>();
        if (rc != SQLITE_ROW) return db_error(rc, "select tile");
        Row r;
        r.hash = q.col_hash(0);
        r.header = q.col_blob(1);
        r.pinned = q.col_i64(2) != 0;
        r.size = static_cast<std::uint64_t>(q.col_i64(3));
        r.is_file = q.col_i64(4) != 0;
        if (!r.is_file) r.data = q.col_blob(5);
        return std::optional<Row>(std::move(r));
    }

    // Checks header and body against each other and the address.
    static std::optional<StoredTile> verify(const TileAddress& a, Row& row, std::vector<std::uint8_t> body) {
        if (body.size() != row.size || content_hash(body) != row.hash) return std::nullopt;
        std::size_t consumed = 0;
        auto h = decode_header(row.header, &consumed);
        if (!h || consumed != row.header.size() || h->body_hash != row.hash || h->body_size != body.size() ||
            h->layer != a.layer || h->key != a.key) {
            return std::nullopt;
        }
        StoredTile t;
        t.header = std::move(*h);
        t.header_bytes = std::move(row.header);
        t.body = std::move(body);
        t.pinned = row.pinned;
        return t;
    }
};

// ---------------------------------------------------------------------------

TileStore::TileStore(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
TileStore::~TileStore() = default;

const TileStoreConfig& TileStore::config() const { return impl_->cfg; }

Result<std::unique_ptr<TileStore>> TileStore::open(const TileStoreConfig& config) {
    if (!valid_store_name(config.name)) return make_error(ErrorCode::InvalidArgument, "tile store: invalid name");
    std::error_code ec;
    std::filesystem::create_directories(config.dir, ec);
    if (ec) return make_error(ErrorCode::Io, "tile store: cannot create " + path_utf8(config.dir));
    auto impl = std::make_unique<Impl>();
    impl->cfg = config;
    impl->blob_dir = config.dir / (config.name + "_blobs");
    const std::string db_path = path_utf8(config.dir / (config.name + ".sqlite3"));
    const int rc = sqlite3_open_v2(db_path.c_str(), &impl->db,
                                   SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) return impl->db_error(rc, "open");
    sqlite3_busy_timeout(impl->db, config.busy_timeout_ms);
    sqlite3_extended_result_codes(impl->db, 0);
    if (auto r = impl->exec("PRAGMA journal_mode = WAL"); !r) return r.error();
    if (auto r = impl->exec("PRAGMA synchronous = NORMAL"); !r) return r.error();
    {
        Impl::Txn txn(*impl);
        if (auto r = txn.begin(); !r) return r.error();
        std::int64_t version = 0;
        {
            Impl::Query q(*impl, "PRAGMA user_version");
            if (q.step() != SQLITE_ROW) return impl->db_error(SQLITE_ERROR, "user_version");
            version = q.col_i64(0);
        }
        if (version == 0) {
            if (auto r = impl->exec(kSchema); !r) return r.error();
        } else if (version != kSchemaVersion) {
            return make_error(ErrorCode::Unsupported, "tile store: schema version " + std::to_string(version));
        }
        if (auto r = txn.commit(); !r) return r.error();
    }
    return std::unique_ptr<TileStore>(new TileStore(std::move(impl)));
}

Result<void> TileStore::put(const TileAddress& addr, TileHeader header, std::span<const std::uint8_t> body, bool pin) {
    if (auto r = check_address(addr); !r) return r;
    if (header.layer != addr.layer || header.key != addr.key) {
        return make_error(ErrorCode::InvalidArgument, "tile store: header does not match the address");
    }
    header.body_hash = content_hash(body);
    header.body_size = body.size();
    auto hb = encode_header(header);
    if (!hb) return hb.error();
    const ContentHash& h = header.body_hash;

    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    Impl::Txn txn(I);
    if (auto r = txn.begin(); !r) return r;

    std::optional<ContentHash> old_hash;
    bool old_pinned = false;
    {
        Impl::Query q(I, "SELECT hash, pinned FROM tiles WHERE scope = ?1 AND layer = ?2 AND key = ?3");
        q.addr(addr);
        const int rc = q.step();
        if (rc == SQLITE_ROW) {
            old_hash = q.col_hash(0);
            old_pinned = q.col_i64(1) != 0;
        } else if (rc != SQLITE_DONE) {
            return I.db_error(rc, "select tile");
        }
    }
    // The body row.
    bool exists = false, is_file = false;
    std::uint64_t stored_size = 0;
    {
        Impl::Query q(I, "SELECT size, data IS NULL FROM bodies WHERE hash = ?1");
        q.blob(1, h);
        const int rc = q.step();
        if (rc == SQLITE_ROW) {
            exists = true;
            stored_size = static_cast<std::uint64_t>(q.col_i64(0));
            is_file = q.col_i64(1) != 0;
        } else if (rc != SQLITE_DONE) {
            return I.db_error(rc, "select body");
        }
    }
    if (!exists) {
        const bool to_file = body.size() > I.cfg.inline_limit;
        if (to_file) {
            if (auto r = I.write_blob(h, body); !r) return r;
        }
        Impl::Query q(I, "INSERT INTO bodies(hash, size, refs, data) VALUES(?1, ?2, 0, ?3)");
        q.blob(1, h).i64(2, static_cast<std::int64_t>(body.size()));
        if (to_file) {
            q.null(3);
        } else {
            q.blob(3, body);
        }
        if (auto r = I.run(q, "insert body"); !r) return r;
        if (auto r = I.add_body_bytes(static_cast<std::int64_t>(body.size())); !r) return r;
    } else if (is_file && (stored_size != body.size() || !I.blob_ok_size(h, stored_size))) {
        // Self-repair of a lost or truncated blob file: we have the bytes.
        if (auto r = I.write_blob(h, body); !r) return r;
    }

    const bool pinned = pin || old_pinned;
    if (old_hash && *old_hash == h) {
        Impl::Query q(I, "UPDATE tiles SET header = ?4, pinned = ?5, "
                         "last_used = (SELECT COALESCE(MAX(last_used), 0) + 1 FROM tiles) "
                         "WHERE scope = ?1 AND layer = ?2 AND key = ?3");
        q.addr(addr).blob(4, *hb).i64(5, pinned ? 1 : 0);
        if (auto r = I.run(q, "update tile"); !r) return r;
    } else {
        {
            Impl::Query q(I, "UPDATE bodies SET refs = refs + 1 WHERE hash = ?1");
            q.blob(1, h);
            if (auto r = I.run(q, "incref"); !r) return r;
        }
        {
            Impl::Query q(I, "INSERT OR REPLACE INTO tiles(scope, layer, key, hash, header, last_used, pinned) "
                             "VALUES(?1, ?2, ?3, ?4, ?5, (SELECT COALESCE(MAX(last_used), 0) + 1 FROM tiles), ?6)");
            q.addr(addr).blob(4, h).blob(5, *hb).i64(6, pinned ? 1 : 0);
            if (auto r = I.run(q, "insert tile"); !r) return r;
        }
        if (old_hash) {
            if (auto r = I.decref(*old_hash); !r) return r;
        }
    }
    if (I.cfg.size_cap_bytes > 0) {
        if (auto r = I.evict_locked(I.cfg.size_cap_bytes, &addr); !r) return r.error();
    }
    return txn.commit();
}

Result<void> TileStore::put_container(const TileAddress& addr, std::span<const std::uint8_t> container, bool pin) {
    auto v = parse_container(container);
    if (!v) return v.error();
    return put(addr, v->header, v->body, pin);
}

Result<std::optional<StoredTile>> TileStore::get(const TileAddress& addr) {
    if (auto r = check_address(addr); !r) return r.error();
    Impl& I = *impl_;
    std::optional<Impl::Row> row;
    {
        std::lock_guard lock(I.mu);
        auto r = I.select_row(addr);
        if (!r) return r.error();
        if (!*r) return std::optional<StoredTile>();
        row = std::move(**r);
        (void)I.touch(addr); // best effort: a busy database only loses one LRU update
    }
    // Hash and read files outside the lock.
    std::optional<std::vector<std::uint8_t>> body;
    if (row->is_file) {
        body = I.read_blob(row->hash, row->size);
    } else {
        body = std::move(row->data);
    }
    if (body) {
        if (auto t = Impl::verify(addr, *row, std::move(*body))) return std::optional<StoredTile>(std::move(*t));
    }
    // Failed: re-check under the write lock before dropping anything.
    std::lock_guard lock(I.mu);
    Impl::Txn txn(I);
    if (auto r = txn.begin(); !r) return r.error();
    auto again = I.select_row(addr);
    if (!again) return again.error();
    if (!*again || (*again)->hash != row->hash) {
        if (auto r = txn.commit(); !r) return r.error();
        return std::optional<StoredTile>(); // removed or replaced concurrently: a plain miss
    }
    Impl::Row& r2 = **again;
    std::optional<std::vector<std::uint8_t>> body2;
    if (r2.is_file) {
        body2 = I.read_blob(r2.hash, r2.size);
    } else {
        body2 = std::move(r2.data);
    }
    const bool body_bad = !body2 || content_hash(*body2) != r2.hash;
    if (body2) {
        if (auto t = Impl::verify(addr, r2, std::move(*body2))) {
            if (auto r = txn.commit(); !r) return r.error();
            return std::optional<StoredTile>(std::move(*t));
        }
    }
    std::uint64_t dropped = 1;
    if (body_bad) {
        auto n = I.drop_body(r2.hash); // bad for every address that shares it
        if (!n) return n.error();
        dropped = *n;
    } else if (auto r = I.drop_entry(addr); !r) {
        return r.error(); // only this entry's header is bad
    }
    if (auto r = txn.commit(); !r) return r.error();
    I.integrity_drops.fetch_add(dropped, std::memory_order_relaxed);
    return std::optional<StoredTile>();
}

Result<std::optional<ContentHash>> TileStore::peek_hash(const TileAddress& addr) {
    if (auto r = check_address(addr); !r) return r.error();
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    Impl::Query q(I, "SELECT hash FROM tiles WHERE scope = ?1 AND layer = ?2 AND key = ?3");
    q.addr(addr);
    const int rc = q.step();
    if (rc == SQLITE_DONE) return std::optional<ContentHash>();
    if (rc != SQLITE_ROW) return I.db_error(rc, "peek");
    return std::optional<ContentHash>(q.col_hash(0));
}

Result<bool> TileStore::set_pinned(const TileAddress& addr, bool pinned) {
    if (auto r = check_address(addr); !r) return r.error();
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    Impl::Query q(I, "UPDATE tiles SET pinned = ?4 WHERE scope = ?1 AND layer = ?2 AND key = ?3");
    q.addr(addr).i64(4, pinned ? 1 : 0);
    if (auto r = I.run(q, "pin"); !r) return r.error();
    return sqlite3_changes(I.db) > 0;
}

Result<std::uint64_t> TileStore::set_pinned_range(const std::string& scope, const std::string& layer, const TileKey& lo,
                                                  const TileKey& hi, bool pinned) {
    if (auto r = check_address(TileAddress{scope, layer, lo}); !r) return r.error();
    if (!hi.valid() || hi.zone != lo.zone || hi.level != lo.level || hi.x < lo.x || hi.y < lo.y) {
        return make_error(ErrorCode::InvalidArgument, "tile store: bad pin range");
    }
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    // x occupies the low 24 bits of the packed key (x + 2^23); rows y lo..hi
    // are one contiguous packed range.
    TileKey first = lo, last = hi;
    first.x = TileKey::kIndexMin;
    last.x = TileKey::kIndexMax;
    Impl::Query q(I, "UPDATE tiles SET pinned = ?6 WHERE scope = ?1 AND layer = ?2 AND key BETWEEN ?3 AND ?4 "
                     "AND (key & 16777215) BETWEEN ?5 AND ?7");
    q.text(1, scope).text(2, layer);
    q.i64(3, static_cast<std::int64_t>(first.packed())).i64(4, static_cast<std::int64_t>(last.packed()));
    q.i64(5, static_cast<std::int64_t>(lo.x) + (std::int64_t{1} << 23));
    q.i64(7, static_cast<std::int64_t>(hi.x) + (std::int64_t{1} << 23));
    q.i64(6, pinned ? 1 : 0);
    if (auto r = I.run(q, "pin range"); !r) return r.error();
    return static_cast<std::uint64_t>(sqlite3_changes(I.db));
}

Result<bool> TileStore::remove(const TileAddress& addr) {
    if (auto r = check_address(addr); !r) return r.error();
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    Impl::Txn txn(I);
    if (auto r = txn.begin(); !r) return r.error();
    auto h = I.delete_row(addr);
    if (!h) return h.error();
    if (*h) {
        if (auto r = I.decref(**h); !r) return r.error();
    }
    if (auto r = txn.commit(); !r) return r.error();
    return h->has_value();
}

Result<std::uint64_t> TileStore::evict_to(std::uint64_t target_bytes) {
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    Impl::Txn txn(I);
    if (auto r = txn.begin(); !r) return r.error();
    auto n = I.evict_locked(target_bytes, nullptr);
    if (!n) return n.error();
    if (auto r = txn.commit(); !r) return r.error();
    return *n;
}

Result<TileStoreStats> TileStore::stats() {
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    TileStoreStats s;
    {
        Impl::Query q(I, "SELECT COUNT(*), COALESCE(SUM(pinned), 0) FROM tiles");
        if (q.step() != SQLITE_ROW) return I.db_error(SQLITE_ERROR, "stats");
        s.tiles = static_cast<std::uint64_t>(q.col_i64(0));
        s.pinned_tiles = static_cast<std::uint64_t>(q.col_i64(1));
    }
    {
        Impl::Query q(I, "SELECT COUNT(*), COALESCE(SUM(data IS NULL), 0) FROM bodies");
        if (q.step() != SQLITE_ROW) return I.db_error(SQLITE_ERROR, "stats");
        s.bodies = static_cast<std::uint64_t>(q.col_i64(0));
        s.blob_files = static_cast<std::uint64_t>(q.col_i64(1));
    }
    auto b = I.body_bytes();
    if (!b) return b.error();
    s.body_bytes = static_cast<std::uint64_t>(*b);
    s.integrity_drops = I.integrity_drops.load(std::memory_order_relaxed);
    s.evicted_tiles = I.evicted.load(std::memory_order_relaxed);
    return s;
}

Result<std::vector<TileIndexEntry>> TileStore::index(const std::string& scope) {
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    Impl::Query q(I, "SELECT layer, key, hash FROM tiles WHERE scope = ?1 ORDER BY layer, key");
    q.text(1, scope);
    std::vector<TileIndexEntry> out;
    int rc = 0;
    while ((rc = q.step()) == SQLITE_ROW) {
        auto key = TileKey::unpack(static_cast<std::uint64_t>(q.col_i64(1)));
        if (!key) return make_error(ErrorCode::Malformed, "tile store: bad key in index");
        out.push_back(TileIndexEntry{q.col_text(0), *key, q.col_hash(2)});
    }
    if (rc != SQLITE_DONE) return I.db_error(rc, "index");
    return out;
}

Result<void> TileStore::check_consistency() {
    Impl& I = *impl_;
    std::lock_guard lock(I.mu);
    const auto bad = [](const std::string& what) { return make_error(ErrorCode::Malformed, "tile store: " + what); };
    std::int64_t sum = 0;
    {
        Impl::Query q(I, "SELECT b.hash, b.size, b.refs, b.data IS NULL, "
                         "(SELECT COUNT(*) FROM tiles t WHERE t.hash = b.hash) FROM bodies b ORDER BY b.hash");
        int rc = 0;
        while ((rc = q.step()) == SQLITE_ROW) {
            const ContentHash h = q.col_hash(0);
            const std::int64_t size = q.col_i64(1);
            if (q.col_i64(2) != q.col_i64(4)) return bad("refs mismatch for " + to_hex(h));
            if (q.col_i64(2) <= 0) return bad("unreferenced body " + to_hex(h));
            if (q.col_i64(3) != 0 && !I.blob_ok_size(h, static_cast<std::uint64_t>(size))) {
                return bad("blob file missing or wrong size: " + to_hex(h));
            }
            sum += size;
        }
        if (rc != SQLITE_DONE) return I.db_error(rc, "check");
    }
    {
        Impl::Query q(I, "SELECT COUNT(*) FROM tiles WHERE hash NOT IN (SELECT hash FROM bodies)");
        if (q.step() != SQLITE_ROW) return I.db_error(SQLITE_ERROR, "check");
        if (q.col_i64(0) != 0) return bad("tiles without a body");
    }
    auto total = I.body_bytes();
    if (!total) return total.error();
    if (*total != sum) return bad("body_bytes " + std::to_string(*total) + " != " + std::to_string(sum));
    return {};
}

} // namespace g2m
