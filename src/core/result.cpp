// src/core/result.cpp - see g2m/core/result.h.

#include "g2m/core/result.h"

namespace g2m {

const char* to_string(ErrorCode code) {
    switch (code) {
    case ErrorCode::Truncated: return "truncated";
    case ErrorCode::BadMagic: return "bad magic";
    case ErrorCode::Unsupported: return "unsupported";
    case ErrorCode::Malformed: return "malformed";
    case ErrorCode::LimitExceeded: return "limit exceeded";
    case ErrorCode::HashMismatch: return "hash mismatch";
    case ErrorCode::InvalidArgument: return "invalid argument";
    case ErrorCode::NotFound: return "not found";
    case ErrorCode::Io: return "i/o error";
    case ErrorCode::Busy: return "busy";
    case ErrorCode::ReadOnly: return "read only";
    }
    return "unknown";
}

} // namespace g2m
