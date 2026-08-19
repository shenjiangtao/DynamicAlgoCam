// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_frame.cpp — DynalgoFrameSet implementation.
//
// [文件说明 / File Description]
// 中文：DynalgoFrameSet实现，提供帧集合的访问和管理功能
// English: DynalgoFrameSet implementation, provides frame set access and management

#include "dynalgo_frame.hpp"

namespace dynalgo {

namespace {

// [辅助函数 / Helper Function]
// 中文：将帧类型转换为数组索引
// English: Convert frame type to array index
inline size_t idx(DynalgoFrameType t) { return static_cast<size_t>(t); }

}

// [方法说明 / Method Description]
// 中文：根据帧类型获取帧指针，不存在返回nullptr
// English: Get frame pointer by type, returns nullptr if not present
DynalgoFrame* DynalgoFrameSet::getFrame(DynalgoFrameType type) {
    if (!present_.test(idx(type)))
        return nullptr;
    return &frames_[idx(type)];
}

// [方法说明 / Method Description]
// 中文：根据帧类型获取常量帧指针，不存在返回nullptr
// English: Get constant frame pointer by type, returns nullptr if not present
const DynalgoFrame* DynalgoFrameSet::getFrame(DynalgoFrameType type) const {
    if (!present_.test(idx(type)))
        return nullptr;
    return &frames_[idx(type)];
}

// [方法说明 / Method Description]
// 中文：存储帧到指定类型槽位，接管所有权
// English: Store frame to specified type slot, takes ownership
void DynalgoFrameSet::setFrame(DynalgoFrameType type, DynalgoFrame frame) {
    frame.type = type;
    frames_[idx(type)] = std::move(frame);
    present_.set(idx(type));
}

} // namespace dynalgo
