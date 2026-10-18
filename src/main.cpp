#include "app/Application.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char** argv) {
    sicdm::AppOptions options;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            options.screenshotPath = argv[++i];
        } else if (std::strcmp(argv[i], "--size") == 0 && i + 2 < argc) {
            options.width = std::atoi(argv[++i]);
            options.height = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::printf("Usage: %s [--size W H] [--screenshot out.ppm]\n", argv[0]);
            return 0;
        }
        // Unknown args (e.g. -NSDocumentRevisionsDebugMode from Xcode) are ignored.
    }

    sicdm::Application app(options);
    return app.run();
}
