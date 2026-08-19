// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_config_validator.cpp — Factory for creating vendor-specific validators instances.
//
// [文件说明 / File Description]
// 中文：供应商特定验证器实例的工厂，根据供应商类型创建相应的验证器
// English: Factory for creating vendor-specific validators instances, creates appropriate validator based on vendor type

#include "dynalgo_config_validator.hpp"

#ifdef ENABLE_ORBBEC
#include "../driver/orbbec/dynalgo_ob_validator.hpp"
#endif

#ifdef ENABLE_RS_AC1
#include "../driver/robosense/dynalgo_rs_validator.hpp"
#endif

namespace dynalgo {

// [工厂函数 / Factory Function]
// 中文：为给定供应商创建验证器实例，不支持的供应商返回nullptr
// English: Create validator instance for given vendor, returns nullptr for unsupported vendors
std::unique_ptr<ConfigValidator> createValidator(DriverVendor vendor) {
    switch (vendor) {
    case DriverVendor::ORBBEC:
#ifdef ENABLE_ORBBEC
        return std::make_unique<ObValidator>();
#else
        return nullptr;
#endif
    case DriverVendor::ROBOSENSE:
#ifdef ENABLE_RS_AC1
        return std::make_unique<RsValidator>();
#else
        return nullptr;
#endif
    case DriverVendor::ALL:
    default:
        return nullptr;
    }
}

} // namespace dynalgo
