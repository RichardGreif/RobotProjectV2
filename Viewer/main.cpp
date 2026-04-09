#include <QApplication>
#include <QString>

#include "MapWidget.h"
#include "SnapshotTcpServer.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    constexpr quint16 Port = 4242;

    SLAM::MapWidget window;
    window.setWindowTitle(QString("MapSnapshot TCP Listener - Port %1").arg(Port));
    window.resize(800, 800);
    window.SetPixelsPerMeter(140.0f);
    window.SetGridSpacingMeters(0.5f);
    window.show();

    SnapshotTcpServer server(Port);
    server.SetSnapshotHandler([&window](const SLAM::MapSnapshot& snapshot)
    {
        window.SetSnapshot(snapshot);
    });

    server.SetStatusHandler([&window](const QString& status)
    {
        window.setWindowTitle(QString("MapSnapshot TCP Listener - %1").arg(status));
    });

    QString errorMessage;
    if (!server.Start(&errorMessage))
    {
        window.setWindowTitle(QString("TCP start failed: %1").arg(errorMessage));
    }

    return app.exec();
}
