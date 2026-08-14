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

enum class PcdMode {
    Stream,
    Single
};

struct CaptureConfig
{
    std::vector<std::string> cameraFilter;
    std::string saveDir;
    float alpha = 0.5f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    bool enableFusion = true;
    bool noFusion = false;
    bool noShow = false;
    bool depthToPcd = false;
    PcdMode pcdMode = PcdMode::Single;
};

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
              << "  --help        Show this help\n"
              << "\nExamples:\n"
              << "  " << prog << "                                 # all devices, default settings\n"
              << "  " << prog << " -c \"305\" \"336L\"             # filter cameras by type\n"
              << "  " << prog << " -s /HDD/dynalgo_capture             # custom save directory\n"
              << "  " << prog << " -c \"305\" --alpha 0.6          # combined options\n"
              << "  " << prog << " --pcd-mode stream             # continuous .pcs stream instead of per-frame .pcd\n"
              << "  " << prog << " --depth-to-pcd                 # convert depth to point cloud and record\n"
              << std::endl;
}

inline CaptureConfig parseArgs(int argc, char** argv) {
    CaptureConfig cfg;
    static struct option longOpts[] = { { "alpha", required_argument, nullptr, 'a' },
                                        { "depth-min", required_argument, nullptr, 'm' },
                                        { "depth-max", required_argument, nullptr, 'x' },
                                        { "no-fusion", no_argument, nullptr, 'n' },
                                        { "no-show", no_argument, nullptr, 'S' },
                                        { "depth-to-pcd", no_argument, nullptr, 'P' },
                                        { "pcd-mode", required_argument, nullptr, 'p' },
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
