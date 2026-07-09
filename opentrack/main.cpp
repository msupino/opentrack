#include "gui/init.hpp"
#include "main-window.hpp"

#include <QDebug>
#include <QTimer>

#include <string_view>

#if defined _WIN32
#   include <windows.h>
#endif

#ifdef __clang__
#   pragma GCC diagnostic ignored "-Wmain"
#endif

static bool force_start_requested(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string_view arg(argv[i] ? argv[i] : "");
        if (arg == "--start" || arg == "--start-tracking")
            return true;
    }
    return false;
}

int main(int argc, char** argv)
{
    const bool force_start = force_start_requested(argc, argv);

    return run_application(argc, argv, [force_start] {
        auto window = std::make_unique<main_window>();
        if (force_start)
        {
            qDebug() << "opentrack: --start requested";
            QTimer::singleShot(0, window.get(), &main_window::start_tracker_);
        }
        return window;
    });
}

#if defined _MSC_VER

int CALLBACK WinMain(HINSTANCE, HINSTANCE, LPSTR, int /* nCmdShow */)
{
    return main(__argc, __argv);
}
#endif
