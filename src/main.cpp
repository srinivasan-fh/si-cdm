#include "app/Application.hpp"
#include "core/CrashReport.hpp"
#include "core/Preferences.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>

int main(int argc, char** argv) {
    sicdm::AppOptions options;
    options.executablePath = argc > 0 ? argv[0] : "";
    options.splash = sicdm::preferences().splash; // the SPLASH preference (--no-splash turns it off for one run)
    bool sizeGiven = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            options.screenshotPath = argv[++i];
        } else if (std::strcmp(argv[i], "--no-splash") == 0) {
            options.splash = false;
        } else if (std::strcmp(argv[i], "--splash-seconds") == 0 && i + 1 < argc) {
            options.splashSeconds = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--2d") == 0) {
            options.start2D = true;
        } else if (std::strcmp(argv[i], "--mcp-port") == 0 && i + 1 < argc) {
            options.mcpPort = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--language") == 0 && i + 1 < argc) {
            options.language = argv[++i];
        } else if (std::strcmp(argv[i], "--command") == 0 && i + 1 < argc) {
            options.commands.push_back(argv[++i]);
        } else if (std::strcmp(argv[i], "--size") == 0 && i + 2 < argc) {
            options.width = std::atoi(argv[++i]);
            options.height = std::atoi(argv[++i]);
            sizeGiven = true;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::printf("Usage: %s [file.sicdm | file.stl | image.png ...] [--size W H] [--2d] [--mcp-port N]\n"
                        "          [--no-splash] [--splash-seconds S] [--screenshot out.ppm] [--command \"LINE\" ...]\n"
                        "          [--language CODE]  (this run only; PREFERENCES SET language CODE keeps it)\n",
                        argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            options.openPath = argv[i];
        }
        // Unknown args (e.g. -NSDocumentRevisionsDebugMode from Xcode) are ignored.
    }

    // The window as it was left, unless a size is given (or this run makes test pictures).
    if (const sicdm::Preferences& p = sicdm::preferences(); !sizeGiven && options.screenshotPath.empty() && p.windowWidth > 0) {
        options.width = p.windowWidth;
        options.height = p.windowHeight;
        options.restoreGeometry = true;
    }

    sicdm::installCrashHandler(SICDM_VERSION); // a fault from here on leaves a report
    try {
        sicdm::Application app(options);
        return app.run();
    } catch (const std::exception& e) {
        sicdm::crashOnUncaught(e.what());
    }
}
