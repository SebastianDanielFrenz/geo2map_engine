// Street fidelity P2: the data-table classifier that maps ESA WorldCover classes to LandClass and a default
// SurfaceKind (data/classifiers/esa_worldcover_v200.json, design 6.5 and 8.2). It is the "base class" of every
// terrain.class sample that no road or junction owns. Pure data: no code branch decides a class (PLAN.md 2.5).
// The table SHA-256 is IClassifier::table_hash and enters the terrain.class config hash. Embedded at build time.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/classifier.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace g2m {
class LandcoverClassifier final : public IClassifier {
public:
    struct Entry {
        std::uint8_t code = 0;                       // class value in the scheme (ESA: 10, 20, ... 100)
        LandClass land_class = LandClass::Unknown;
        SurfaceKind surface = SurfaceKind::Unknown;
        friend bool operator==(const Entry&, const Entry&) = default;
    };

    // Strict loader: format "g2m.classifier_table/1", every one of the 11 ESA WorldCover v200 codes exactly once,
    // known land class and surface names, unknown keys rejected.
    static Result<LandcoverClassifier> load(std::span<const std::byte> json);

    std::string_view name() const override { return name_; }
    ContentHash table_hash() const override { return hash_; }
    // Facts of another scheme, or a code without an entry (including the scheme's no-data value 0), classify as Unknown.
    LandClass land_class(const ClassifierFacts& facts) const override;
    SurfaceKind surface(const ClassifierFacts& facts) const override;

    std::string_view scheme() const { return scheme_; }
    std::span<const Entry> entries() const { return entries_; } // ascending code
    const Entry* find(std::uint8_t code) const;

private:
    LandcoverClassifier() = default;
    std::string name_, scheme_;
    ContentHash hash_{};
    std::array<Entry, 11> entries_{};
};

// The embedded data/classifiers/esa_worldcover_v200.json, parsed once. A corrupt build aborts.
const LandcoverClassifier& builtin_esa_worldcover_classifier();
std::span<const std::byte> builtin_esa_worldcover_classifier_bytes();
} // namespace g2m
