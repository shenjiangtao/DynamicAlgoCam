// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_capture_config.hpp — Capture configuration: CLI parsing, config struct.

#pragma once

#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>

namespace dynalgo {

// [枚举说明 / Enum Description]
// 中文: 点云保存模式
// English: Pointcloud save mode
enum class PcdMode {
    Stream,  // 中文: 连续流模式 / English: Continuous stream mode
    Single   // 中文: 单帧文件模式 / English: Single frame file mode
};

// [结构体说明 / Struct Description]
// 中文: 采集配置结构体，包含CLI解析后的所有配置参数
// English: Capture configuration struct, contains all parsed CLI parameters
struct CaptureConfig
{
    // 中文: 相机类型过滤器 / English: Camera type filter
    std::vector<std::string> cameraFilter;
    // 中文: 保存目录 / English: Save directory
    std::string saveDir;
    // 中文: 深度叠加透明度 0.0-1.0 / English: Depth overlay opacity 0.0-1.0
    float alpha = 0.5f;
    // 中文: 深度色图最小值(米) / English: Min depth in meters for colormap
    float depthMinM = 0.3f;
    // 中文: 深度色图最大值(米) / English: Max depth in meters for colormap
    float depthMaxM = 5.0f;
    // 中文: 是否启用融合 / English: Enable fusion
    bool enableFusion = true;
    // 中文: 禁用融合标志 / English: Disable fusion flag
    bool noFusion = false;
    // 中文: 禁用SDL预览 / English: Disable SDL preview
    bool noShow = false;
    // 中文: 深度转点云标志 / English: Depth to pointcloud flag
    bool depthToPcd = false;
    // 中文: 点云保存模式 / English: Pointcloud save mode
    PcdMode pcdMode = PcdMode::Single;

    // Engagement loop (Phase C)
    // 中文: 模型后端类型 / English: Model backend type
    std::string engageModel;    // e.g. "DUMMY", "YOLOV8_PY" — empty = disabled
    // 中文: 执行器类型 / English: Actuator type
    std::string engageActuator; // e.g. "DUMMY" — empty = disabled
    // 中文: 模型权重/配置路径 / English: Model weights/config path
    std::string engageModelPath; // model weights / config path
};

// [函数说明 / Function Description]
// 中文: 打印使用帮助
// English: Print usage help
inline void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options] [camera_name_filter...]\n"
              << "\nOptions:\n"
              << "  -c <name...>  Camera type filter (can specify multiple, e.g. -c \"305\" \"336L\")\n"
              << "  -s <dir>      Save directory (default: capture_output/)\n"
              << "  --alpha VAL   Depth overlay opacity 0.0-1.0 (default: 0.5)\n"
              << "  --depth-min M Min depth in meters for colormap (default: 0.3)\n"
              << "  --depth-max M Max depth in meters for colormap (default: 5.0)\n"
              << "  --no-fusion   Disable D2C fusion output (only save individual streams)\n"
              << "  --no-show     Disable SDL live preview window\n"
              << "  --depth-to-pcd  Convert depth frames to point cloud and record (default: off)\n"
              << "  --pcd-mode M  Point cloud save mode: single (per-frame .pcd, default) or stream (single .pcs)\n"
              << "  --engage-model TYPE  Enable engagement loop with model backend (DUMMY, YOLOV8_PY, etc.)\n"
              << "  --engage-actuator TYPE  Enable engagement loop with actuator (DUMMY, etc.)\n"
              << "  --engage-model-path PATH  Model weights/config path for engage-model\n"
              << "  --help        Show this help\n"
              << "\nExamples:\n"
              << "  " << prog << "                                 # all devices, default settings\n"
              << "  " << prog << " -c \"305\" \"336L\"             # filter cameras by type\n"
              << "  " << prog << " -s /HDD/dynalgo_capture             # custom save directory\n"
              << "  " << prog << " -c \"305\" --alpha 0.6          # combined options\n"
              << "  " << prog << " --pcd-mode stream             # continuous .pcs stream instead of per-frame .pcd\n"
              << "  " << prog << " --depth-to-pcd                 # convert depth to point cloud and record\n"
              << "  " << prog << " --engage-model DUMMY --engage-actuator DUMMY  # dry-run engagement loop\n"
              << std::endl;
}

// [函数说明 / Function Description]
// 中文: 解析命令行参数
// English: Parse command line arguments
inline CaptureConfig parseArgs(int argc, char** argv) {
    CaptureConfig cfg;
    static struct option longOpts[] = { { "alpha", required_argument, nullptr, 'a' },
                                        { "depth-min", required_argument, nullptr, 'm' },
                                        { "depth-max", required_argument, nullptr, 'x' },
                                        { "no-fusion", no_argument, nullptr, 'n' },
                                        { "no-show", no_argument, nullptr, 'S' },
                                        { "depth-to-pcd", no_argument, nullptr, 'P' },
                                        { "pcd-mode", required_argument, nullptr, 'p' },
                                        { "engage-model", required_argument, nullptr, 1000 },
                                        { "engage-actuator", required_argument, nullptr, 1001 },
                                        { "engage-model-path", required_argument, nullptr, 1002 },
                                        { "help", no_argument, nullptr, 'h' },
                                        { nullptr, 0, nullptr, 0 } };

    opterr = 0;
    int ch;
    int optIdx = 0;

    while ((ch = getopt_long(argc, argv, "c:s:h", longOpts, &optIdx)) != -1) {
        switch (ch) {
        case 'c':
            cfg.cameraFilter.push_back(optarg);
            while (optind < argc && argv[optind][0] != '-')
                cfg.cameraFilter.push_back(argv[optind++]);
            break;
        case 's':
            cfg.saveDir = optarg;
            while (!cfg.saveDir.empty() && cfg.saveDir.back() == '/')
                cfg.saveDir.pop_back();
            break;
        case 'a':
            cfg.alpha = std::stof(optarg);
            cfg.alpha = std::max(0.0f, std::min(1.0f, cfg.alpha));
            break;
        case 'm':
            cfg.depthMinM = std::stof(optarg);
            break;
        case 'x':
            cfg.depthMaxM = std::stof(optarg);
            break;
        case 'n':
            cfg.noFusion = true;
            break;
        case 'S':
            cfg.noShow = true;
            break;
        case 'P':
            cfg.depthToPcd = true;
            break;
        case 'p':
            if (std::string(optarg) == "stream")
                cfg.pcdMode = PcdMode::Stream;
            else
                cfg.pcdMode = PcdMode::Single;
            break;
        case 1000: // --engage-model
            cfg.engageModel = optarg;
            break;
        case 1001: // --engage-actuator
            cfg.engageActuator = optarg;
            break;
        case 1002: // --engage-model-path
            cfg.engageModelPath = optarg;
            break;
        case 'h':
            printUsage(argv[0]);
            exit(0);
        default:
            break;
        }
    }
    for (int i = optind; i < argc; i++)
        cfg.cameraFilter.push_back(argv[i]);

    if (cfg.noFusion)
        cfg.enableFusion = false;

    return cfg;
}

} // namespace dynalgo
