// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_frame.cpp — DynalgoFrameSet implementation.

#include "dynalgo_frame.hpp"

namespace dynalgo {

namespace {
inline size_t idx(DynalgoFrameType t) { return static_cast<size_t>(t); }
}

DynalgoFrame* DynalgoFrameSet::getFrame(DynalgoFrameType type) {
    if (!present_.test(idx(type)))
        return nullptr;
    return &frames_[idx(type)];
}

const DynalgoFrame* DynalgoFrameSet::getFrame(DynalgoFrameType type) const {
    if (!present_.test(idx(type)))
        return nullptr;
    return &frames_[idx(type)];
}

void DynalgoFrameSet::setFrame(DynalgoFrameType type, DynalgoFrame frame) {
    frame.type = type;
    frames_[idx(type)] = std::move(frame);
    present_.set(idx(type));
}

} // namespace dynalgo
