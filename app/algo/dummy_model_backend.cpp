// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dummy_model_backend.cpp — DummyModelBackend implementation + self-registration.

#include "dummy_model_backend.hpp"
#include "dynalgo_model_factory.hpp"
#include "dynalgo_log.hpp"

namespace dynalgo {

// [方法说明 / Method Description]
// 中文: 加载虚拟模型后端配置
// English: Load dummy model backend configuration
bool DummyModelBackend::load(const DynalgoModelConfig& cfg)
{
    (void)cfg;
    DYNALGO_LOG_INFO_S("[engage] DummyModelBackend load() ok");
    return true;
}

// [方法说明 / Method Description]
// 中文: 执行虚拟模型推理
// English: Run dummy model inference
bool DummyModelBackend::infer(const DynalgoFrame& frame, std::vector<DynalgoDetectionResult>& out)
{
    if (!enabled_)
        return true;

    DynalgoDetectionResult det;
    if (hasFixedDet_) {
        det = fixedDet_;
    } else {
        // Synthesize a detection at frame centre, 10% of frame size
        det.classId = 0;
        det.score = 0.9f;
        det.x = frame.width * 0.45f;
        det.y = frame.height * 0.45f;
        det.w = frame.width * 0.1f;
        det.h = frame.height * 0.1f;
        det.label = "dummy";
    }
    out.push_back(det);
    return true;
}

// [自注册说明 / Self-registration Description]
// 中文: 虚拟模型后端自注册
// English: DummyModelBackend self-registration
namespace {
struct DummyModelBackendRegistrar
{
    DummyModelBackendRegistrar()
    {
        registerModelBackend(DynalgoModelType::DUMMY, []() {
            return std::make_unique<DummyModelBackend>();
        });
    }
} dummyModelBackendRegistrar __attribute__((used));
}

} // namespace dynalgo