// ---------------------------------------------------------------------------
// fiducial-detector-service
//
// Reads camera frames, detects fiducial markers (AprilTag / ArUco), estimates
// each marker's pose in the camera optical frame, and publishes protobuf
// datagrams over UDP.
//
// Runs unmodified on Jetson (JetPack 6) and Raspberry Pi (Bookworm). The only
// platform-varying value is the `gst:` pipeline string in config.
// ---------------------------------------------------------------------------

#include "app_config.h"
#include "aruco_compat.h"
#include "clock_domain.h"
#include "gst_capture.h"
#include "stream_worker.h"

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace
{

std::atomic<bool> g_stop{false};

void signalHandler(int) { g_stop.store(true); }

void printUsage()
{
    std::cout <<
        "fiducial-detector-service: AprilTag/ArUco pose over UDP\n"
        "\n"
        "Usage:\n"
        "  fiducial-detector-service --config <file.yaml>\n"
        "  fiducial-detector-service -c <file.yaml>        validate config and exit\n"
        "\n"
        "Options:\n"
        "  --config <path>   YAML config with a `streams:` list (required)\n"
        "  -c <path>         Parse, fill defaults, validate, report, and exit without\n"
        "                    starting. Exit 0 = usable. For provisioning scripts that\n"
        "                    bootstrap a config before the camera exists.\n"
        "  --no-heal         Do not write defaults back into the config file\n"
        "  --verbose         Log every detection (bench use; noisy)\n"
        "  --version         Print version and library versions\n"
        "  --help            This message\n"
        "\n"
        "Config lookup when --config is omitted, first match wins:\n"
        "  /etc/fiducial-detector-service/config.yaml\n"
        "  ./config/jetson_down.yaml\n"
        "\n"
        "Send SIGINT or SIGTERM to stop.\n";
}

void printVersion()
{
    std::cout << "fiducial-detector-service " << FD_VERSION << "\n"
              << "  OpenCV     " << CV_VERSION << "\n"
              << "  aruco API  " << fd::ArucoDetectorCompat::backend() << "\n"
              << "  GStreamer  " << GST_VERSION_MAJOR << "." << GST_VERSION_MINOR
              << "." << GST_VERSION_MICRO << "\n"
              << "  clock_domain " << fd::clockDomain() << "\n";
}

std::string defaultConfigPath()
{
    const char* candidates[] = {
        "/etc/fiducial-detector-service/config.yaml",
        "config/jetson_down.yaml",
    };
    for (const char* p : candidates)
    {
        FILE* f = std::fopen(p, "r");
        if (f) { std::fclose(f); return p; }
    }
    return {};
}

}  // namespace

int main(int argc, char* argv[])
{
    std::string config_path;
    bool validate_only = false;
    bool heal          = true;
    bool force_verbose = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
            {
                std::cerr << "missing value for " << a << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if      (a == "--config")  config_path = next();
        else if (a == "-c")      { validate_only = true; if (i + 1 < argc && argv[i+1][0] != '-') config_path = next(); }
        else if (a == "--no-heal") heal = false;
        else if (a == "--verbose") force_verbose = true;
        else if (a == "--version") { printVersion(); return 0; }
        else if (a == "--help" || a == "-h") { printUsage(); return 0; }
        else
        {
            std::cerr << "unknown argument: " << a << "\n\n";
            printUsage();
            return 2;
        }
    }

    if (config_path.empty()) config_path = defaultConfigPath();
    if (config_path.empty())
    {
        std::cerr << "[main] no config file. Pass --config <file.yaml> or install one at "
                     "/etc/fiducial-detector-service/config.yaml\n";
        return 2;
    }

    // ---- Load, complete, validate ----------------------------------------
    //
    // Read the config, fill missing keys with documented defaults, and write it
    // back if anything changed.

    fd::AppConfig cfg;
    bool healed = false;
    try
    {
        cfg = fd::AppConfig::loadFromFile(config_path, &healed);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }

    if (force_verbose) cfg.logging.verbose = true;

    if (healed && heal)
    {
        try
        {
            cfg.writeToFile(config_path);
            std::cout << "[main] config was missing keys; defaults written back to "
                      << config_path << "\n";
        }
        catch (const std::exception& e)
        {
            // Not fatal: a read-only /etc runs correctly on the in-memory
            // defaults.
            std::cerr << "[main] could not write healed config (" << e.what()
                      << "); continuing with in-memory defaults\n";
        }
    }

    std::vector<std::string> errors;
    const bool ok = cfg.validate(errors);

    std::cout << "[main] " << config_path << ": " << cfg.summary();

    if (!errors.empty())
    {
        std::cerr << "[main] config problems:\n";
        for (const auto& e : errors) std::cerr << "  - " << e << "\n";
    }

    // `-c` validates and exits, so a provisioning script can check a config
    // without a camera present.
    if (validate_only)
    {
        std::cout << "[main] config " << (ok ? "OK" : "INVALID") << "\n";
        return ok ? 0 : 1;
    }

    if (!ok)
    {
        std::cerr << "[main] refusing to start on an invalid config. "
                     "Run with -c to check it without starting.\n";
        return 1;
    }

    // ---- Start ------------------------------------------------------------

    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);
    // Never terminate on a write to a closed pipe (journald restart, logger
    // gone).
    std::signal(SIGPIPE, SIG_IGN);

    fd::GstCapture::initGst(&argc, &argv);

    std::cout << "[main] fiducial-detector-service " << FD_VERSION
              << "  clock_domain=" << fd::clockDomain() << "\n"
              << "[main] " << fd::ArucoDetectorCompat::backend() << "\n";

    std::vector<std::unique_ptr<fd::StreamWorker>> workers;
    for (const auto& sc : cfg.streams)
    {
        auto w = std::make_unique<fd::StreamWorker>(sc, cfg.udp, cfg.logging, g_stop);
        std::string err;
        if (!w->init(err))
        {
            std::cerr << "[main] stream init failed: " << err << "\n";
            return 1;
        }
        workers.push_back(std::move(w));
    }

    if (workers.size() == 1)
    {
        workers[0]->run();
    }
    else
    {
        // One thread per stream. Streams are independent: a wedged pipeline
        // rebuilds on its own thread without affecting the others.
        std::vector<std::thread> threads;
        threads.reserve(workers.size());
        for (auto& w : workers)
        {
            fd::StreamWorker* raw = w.get();
            threads.emplace_back([raw]() { raw->run(); });
        }
        for (auto& t : threads) t.join();
    }

    std::cout << "[main] stopped\n";
    return 0;
}
