// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame.cpp — NioFrameSet implementation.

#include "nio_frame.hpp"

namespace nio {

namespace {
inline size_t idx(NioFrameType t) { return static_cast<size_t>(t); }
}

NioFrame* NioFrameSet::getFrame(NioFrameType type) {
    if (!present_.test(idx(type)))
        return nullptr;
    return &frames_[idx(type)];
}

const NioFrame* NioFrameSet::getFrame(NioFrameType type) const {
    if (!present_.test(idx(type)))
        return nullptr;
    return &frames_[idx(type)];
}

void NioFrameSet::setFrame(NioFrameType type, NioFrame frame) {
    frame.type = type;
    frames_[idx(type)] = std::move(frame);
    present_.set(idx(type));
}

} // namespace nio
