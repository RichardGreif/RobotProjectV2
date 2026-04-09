#include <QApplication>

#include "MapSnapshot.h"
#include "MapWidget.h"

namespace
{
    SLAM::MapSnapshot CreateDummySnapshot()
    {
        SLAM::MapSnapshot snapshot;

        snapshot.walls[0] = {Vec2(-2.0f, -1.5f), Vec2(2.0f, -1.5f), 6.0f};
        snapshot.walls[1] = {Vec2(2.0f, -1.5f), Vec2(2.0f, 1.5f), 5.0f};
        snapshot.walls[2] = {Vec2(2.0f, 1.5f), Vec2(-2.0f, 1.5f), 4.0f};
        snapshot.walls[3] = {Vec2(-2.0f, 1.5f), Vec2(-2.0f, -1.5f), 5.0f};
        snapshot.walls[4] = {Vec2(-0.5f, -1.5f), Vec2(-0.5f, -0.2f), 3.0f};
        snapshot.walls[5] = {Vec2(0.8f, 0.3f), Vec2(1.7f, 1.1f), 2.0f};
        snapshot.walls[6] = {Vec2(-1.7f, 0.8f), Vec2(-0.8f, 0.8f), 1.5f};
        snapshot.wallCount = 7;

        return snapshot;
    }
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    SLAM::MapWidget window;
    window.setWindowTitle("MapSnapshot Dummy Test");
    window.resize(800, 800);
    window.SetPixelsPerMeter(140.0f);
    window.SetGridSpacingMeters(0.5f);
    window.SetSnapshot(CreateDummySnapshot());
    window.show();

    return app.exec();
}
