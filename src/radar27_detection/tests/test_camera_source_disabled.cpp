#include "radar27_detection/input/camera_source.hpp"
#include <iostream>
#include <stdexcept>

using namespace radar27_detection::input;

int main()
{
    try {
        CameraSourceConfig config;
        config.serial_number = "test-camera";
        for (const auto* brand : {"hik", "daheng"}) {
            config.brand = brand;
            CameraSource source(config);
            if (source.read().status != ReadStatus::Error) throw std::runtime_error("read before open");
            for (int attempt = 0; attempt < 2; ++attempt) {
                bool rejected = false;
                try { source.open(); } catch (const std::runtime_error& error) {
                    rejected = std::string(error.what()).find("not compiled") != std::string::npos;
                }
                if (!rejected) throw std::runtime_error("missing SDK diagnostic or leaked session lock");
                source.close();
                source.close();
            }
        }
        config.brand = "unknown";
        bool rejected = false;
        try { CameraSource source(config); source.open(); }
        catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) throw std::runtime_error("unknown brand accepted");
        std::cout << "PASS: disabled SDK errors, repeat open/close, invalid brand\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
