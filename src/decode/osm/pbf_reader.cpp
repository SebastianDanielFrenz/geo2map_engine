// src/decode/osm/pbf_reader.cpp - see g2m/decode/osm/pbf_reader.h. Every
// osmium/protozero type is confined to this translation unit.

#include "g2m/decode/osm/pbf_reader.h"

// NOMINMAX/WIN32_LEAN_AND_MEAN/_CRT_SECURE_NO_WARNINGS are supplied by the
// g2m_dep_libosmium INTERFACE target (cmake/deps_decoders.cmake) so they
// apply consistently to every TU that includes osmium headers.
#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp> // registers ONLY the PBF format factory - avoids any_input.hpp's
                                    // transitive pull of bzip2_compression.hpp (bzlib.h, not an
                                    // approved dependency and not needed: PBF-only reading, PLAN.md 13)
#include <osmium/osm/entity_bits.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

#include <protozero/pbf_reader.hpp>

#include <cstdint>

namespace g2m::decode::osm {
namespace {

// PBF fileblock framing, spec-fixed limits (OSM PBF format spec / libosmium
// itself): BlobHeader <= 64 KiB, a Blob's declared datasize <= 32 MiB.
constexpr std::uint32_t kMaxBlobHeaderSize = 64u * 1024u;
constexpr std::uint32_t kMaxBlobDataSize = 32u * 1024u * 1024u;

// Validates the raw PBF fileblock framing (the sequence of [4-byte
// big-endian BlobHeader length][BlobHeader][Blob] records) BEFORE handing
// the file to osmium::io::Reader, so a malformed/truncated framing is
// rejected as our own DecodeError instead of reaching libosmium's own
// framing code. G1c step 0 item 4b: a BlobHeader whose `datasize` (field 3)
// claims more bytes than the file actually has left (found via fuzz_pbf on
// a 16-byte crafted input: BlobHeader.datasize=15015 with only 4 bytes of
// file remaining) let libosmium's pbf_input_format.hpp construct a Blob
// buffer around a short/empty read, which then reached a memcpy with a
// null source pointer and a non-zero length (UBSan "null pointer passed as
// argument 2") instead of failing cleanly. This function only needs to
// parse the BlobHeader (a handful of small varint/length-delimited fields);
// the Blob body itself is left for libosmium's real parse, so this is a
// second, cheap, sequential pass over just the file's block headers.
void validate_pbf_framing(const std::string& path) {
    g2m::decode::FileByteSource file_src(path);
    // FileByteSource::read(offset, len, out&) (the pure-virtual override it
    // must declare) hides IByteSource's 2-arg allocate-and-return
    // convenience overload by ordinary C++ name-hiding when called through
    // the concrete type - go through the base interface so both overloads
    // stay visible, matching every other g2m_decode caller (which always
    // holds an IByteSource&/shared_ptr<IByteSource>, never a concrete
    // subclass object).
    const g2m::decode::IByteSource& src = file_src;
    const std::uint64_t size = src.size();
    std::uint64_t pos = 0;
    while (pos < size) {
        if (size - pos < 4) throw DecodeError("osm: truncated PBF fileblock length prefix");
        const auto len_bytes = src.read(pos, 4);
        const auto b = [&](int i) { return static_cast<std::uint32_t>(static_cast<std::uint8_t>(len_bytes[static_cast<std::size_t>(i)])); };
        const std::uint32_t blob_header_len = (b(0) << 24) | (b(1) << 16) | (b(2) << 8) | b(3);
        pos += 4;
        if (blob_header_len == 0 || blob_header_len > kMaxBlobHeaderSize) {
            throw DecodeError("osm: implausible BlobHeader length");
        }
        if (size - pos < blob_header_len) throw DecodeError("osm: truncated BlobHeader");
        const auto header_bytes = src.read(pos, blob_header_len);
        pos += blob_header_len;

        std::int64_t datasize = -1;
        bool have_type = false;
        protozero::pbf_reader msg(reinterpret_cast<const char*>(header_bytes.data()), header_bytes.size());
        while (msg.next()) {
            // BlobHeader.datasize (field 3, required int32) must actually be
            // varint-encoded before get_int32() is called: protozero
            // ASSERTS that (has_wire_type(varint)) internally rather than
            // throwing, so a crafted field 3 with a different wire type
            // (e.g. length-delimited) reaches a raw assert() and aborts the
            // process - not a g2m::decode::DecodeError, escaping every catch
            // boundary just like the framing bugs this function exists to
            // reject. Found by fuzz_pbf (linux-asan-ubsan) on this very
            // validator, a few minutes into step 0 item 5's fuzzing pass.
            if (msg.tag() == 3 && msg.wire_type() == protozero::pbf_wire_type::varint) {
                datasize = msg.get_int32();
            } else if (msg.tag() == 1 && msg.wire_type() == protozero::pbf_wire_type::length_delimited) {
                // BlobHeader.type (field 1, required string): libosmium's own
                // decode_blob_header() (pbf_input_format.hpp) leaves its
                // blob_header_type as a default-constructed (nullptr-data)
                // protozero::data_view when this field never appears at all,
                // then unconditionally std::strncmp()s against it - glibc
                // marks strncmp's second argument nonnull regardless of the
                // length passed, so a BlobHeader with no type field at all
                // reaches a real UBSan "null pointer passed as argument 2"
                // report (pbf_input_format.hpp:171), harmless in practice
                // (the length is 0) but still real UB libosmium relies on the
                // caller never feeding it. Found by fuzz_pbf
                // (linux-asan-ubsan), same fuzzing pass as the datasize
                // wire-type bug above: a 39-byte file whose 11-byte
                // BlobHeader repeats field 3 four times and never has field 1.
                msg.skip();
                have_type = true;
            } else {
                msg.skip();
            }
        }
        if (!have_type) {
            throw DecodeError("osm: BlobHeader missing required type field");
        }
        if (datasize <= 0 || static_cast<std::uint32_t>(datasize) > kMaxBlobDataSize) {
            throw DecodeError("osm: implausible or missing Blob datasize");
        }
        if (size - pos < static_cast<std::uint64_t>(datasize)) {
            throw DecodeError("osm: Blob datasize exceeds bytes remaining in file");
        }
        pos += static_cast<std::uint64_t>(datasize); // Blob body itself: left to libosmium's real parse.
    }
}

std::vector<Tag> convert_tags(const osmium::TagList& tags) {
    std::vector<Tag> out;
    out.reserve(tags.size());
    for (const auto& t : tags) out.push_back(Tag{t.key(), t.value()});
    return out;
}

OsmMemberType convert_member_type(osmium::item_type t) {
    switch (t) {
        case osmium::item_type::node: return OsmMemberType::Node;
        case osmium::item_type::way: return OsmMemberType::Way;
        case osmium::item_type::relation: return OsmMemberType::Relation;
        default: throw DecodeError("osm: relation member with an unsupported type");
    }
}

class Bridge : public osmium::handler::Handler {
public:
    Bridge(PbfHandler& handler, const BoundingBox* bbox) : handler_(handler), bbox_(bbox) {}

    void node(const osmium::Node& n) {
        const osmium::Location loc = n.location();
        if (bbox_ != nullptr) {
            if (!loc.valid()) return;
            if (!bbox_->contains(loc.lat(), loc.lon())) return;
        }
        OsmNode out;
        out.id = n.id();
        if (loc.valid()) {
            out.lon_1e7 = loc.x();
            out.lat_1e7 = loc.y();
        }
        out.tags = convert_tags(n.tags());
        handler_.on_node(out);
        ++stats_.node_count;
    }

    void way(const osmium::Way& w) {
        OsmWay out;
        out.id = w.id();
        out.node_refs.reserve(w.nodes().size());
        for (const auto& nr : w.nodes()) out.node_refs.push_back(nr.ref());
        out.tags = convert_tags(w.tags());
        handler_.on_way(out);
        ++stats_.way_count;
    }

    void relation(const osmium::Relation& r) {
        OsmRelation out;
        out.id = r.id();
        out.members.reserve(r.members().size());
        for (const auto& m : r.members()) {
            out.members.push_back(OsmMember{convert_member_type(m.type()), m.ref(), std::string(m.role())});
        }
        out.tags = convert_tags(r.tags());
        handler_.on_relation(out);
        ++stats_.relation_count;
    }

    const PbfStats& stats() const { return stats_; }

private:
    PbfHandler& handler_;
    const BoundingBox* bbox_;
    PbfStats stats_;
};

} // namespace

PbfStats read_pbf(const std::string& path, PbfHandler& handler, const BoundingBox* bbox) {
    try {
        validate_pbf_framing(path);
        osmium::io::Reader reader(path, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way |
                                             osmium::osm_entity_bits::relation);
        try {
            Bridge bridge(handler, bbox);
            while (osmium::memory::Buffer buffer = reader.read()) {
                osmium::apply(buffer, bridge);
            }
            reader.close();
            return bridge.stats();
        } catch (...) {
            // Always release the reader's file handle / worker thread before
            // propagating, even when reader.read() itself throws mid-parse
            // on malformed content. A skipped close() on that path otherwise
            // leaks one fd (Linux) or handle (Windows) per failed parse
            // (G1c step 0 item 4a, found via fuzz_pbf: "Too many open
            // files" after ~3s of ~2000 failing parses in a row).
            reader.close();
            throw;
        }
    } catch (const DecodeError&) {
        throw;
    } catch (const std::exception& e) {
        throw DecodeError(std::string("osm: ") + e.what());
    }
}

} // namespace g2m::decode::osm
