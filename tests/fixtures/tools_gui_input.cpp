// Test-only input injection: real HighGUI windows/event loop, synthetic pixel clicks.
// Loaded only into test children via LD_PRELOAD, never installed with the nodes.
#include <opencv2/highgui.hpp>
#include <dlfcn.h>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {
cv::MouseCallback callback = nullptr;
void* userdata = nullptr;
std::ifstream commands;
}
namespace cv {
void setMouseCallback(const String& name, MouseCallback cb, void* data) {
    // The callback is intentionally driven here instead of the desktop mouse.
    // HighGUI still creates and paints the real window in the offscreen backend.
    (void)name;
    callback = cb;
    userdata = data;
    commands.close();
    commands.clear();
    commands.open(std::getenv("RADAR27_TEST_INPUT"));
}
int waitKey(int delay) {
    using Function = int (*)(int);
    auto original = reinterpret_cast<Function>(dlsym(RTLD_NEXT, "_ZN2cv7waitKeyEi"));
    original(delay == 1000 ? 1 : delay);
    if (delay == 1000) return -1;
    std::string action;
    if (!(commands >> action)) return 'q';
    if (action == "throw") throw std::runtime_error("injected GUI error");
    if (action == "cancel") return 'q';
    if (action == "click") {
        int x, y;
        commands >> x >> y;
        callback(EVENT_LBUTTONDOWN, x, y, 0, userdata);
    }
    return -1;
}
}
