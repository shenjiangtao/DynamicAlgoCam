// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame.cpp — NioFrameSet implementation.

#include "nio_frame.hpp"

namespace nio {

NioFrame* NioFrameSet::getFrame(NioFrameType type) {
    auto it = frames_.find(type);
    return (it != frames_.end()) ? &it->second : nullptr;
}

const NioFrame* NioFrameSet::getFrame(NioFrameType type) const {
    auto it = frames_.find(type);
    return (it != frames_.end()) ? &it->second : nullptr;
}

void NioFrameSet::setFrame(NioFrameType type, NioFrame frame) {
    frame.type = type;
    frames_[type] = std::move(frame);
}

} // namespace nio
