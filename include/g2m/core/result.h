// g2m/core/result.h - value-or-error return type for code that handles
// untrusted bytes or external resources (decoders, stores, the server).
// No exceptions on these paths: a decoder of server- or file-supplied bytes
// must fail by value (fuzzable, and clang-cl ASan mishandles a caught
// exception object - CLAUDE.md "Windows ASan caveat").
#pragma once

#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

namespace g2m {

enum class ErrorCode : std::uint16_t {
    Truncated = 1,       // input ends before a declared field or section
    BadMagic,            // not this format
    Unsupported,         // a version / encoding / codec this build does not know
    Malformed,           // structurally invalid (non-canonical varint, unsorted ids, bad index, ...)
    LimitExceeded,       // a declared size exceeds a decoder limit
    HashMismatch,        // content does not match its declared hash
    InvalidArgument,     // caller error (encoder input, bad key, bad name)
    NotFound,
    Io,                  // file system / database error
    Busy,                // database locked beyond the busy timeout
    ReadOnly,            // a mutating call on a TileStoreConfig::read_only store
};

const char* to_string(ErrorCode code);

struct Error {
    ErrorCode code = ErrorCode::Malformed;
    std::string message;
};

inline Error make_error(ErrorCode code, std::string message) { return Error{code, std::move(message)}; }

template <class T>
class [[nodiscard]] Result {
public:
    Result(T value) : v_(std::in_place_index<0>, std::move(value)) {}  // NOLINT: implicit by design
    Result(Error error) : v_(std::in_place_index<1>, std::move(error)) {}  // NOLINT

    bool ok() const { return v_.index() == 0; }
    explicit operator bool() const { return ok(); }

    T& value() & {
        assert(ok());
        return std::get<0>(v_);
    }
    const T& value() const& {
        assert(ok());
        return std::get<0>(v_);
    }
    T&& value() && {
        assert(ok());
        return std::get<0>(std::move(v_));
    }
    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }
    T& operator*() & { return value(); }
    const T& operator*() const& { return value(); }

    const Error& error() const {
        assert(!ok());
        return std::get<1>(v_);
    }

private:
    std::variant<T, Error> v_;
};

// Result<void>: success or an error.
template <>
class [[nodiscard]] Result<void> {
public:
    Result() = default;
    Result(Error error) : e_(std::move(error)) {}  // NOLINT

    bool ok() const { return !e_.has_value(); }
    explicit operator bool() const { return ok(); }
    const Error& error() const {
        assert(!ok());
        return *e_;
    }

private:
    std::optional<Error> e_;
};


} // namespace g2m
