#pragma once

#include <QByteArray>
#include <QHash>
#include <QObject>
#include <QString>
#include <QTcpServer>
#include <functional>

#include "MapSnapshot.h"

class QTcpSocket;

class SnapshotTcpServer : public QObject
{
public:
    explicit SnapshotTcpServer(quint16 port, QObject* parent = nullptr);

    bool Start(QString* errorMessage = nullptr);
    quint16 GetPort() const;

    void SetSnapshotHandler(const std::function<void(const SLAM::MapSnapshot&)>& handler);
    void SetStatusHandler(const std::function<void(const QString&)>& handler);

private:
    void HandleNewConnection();
    void HandleSocketReadyRead(QTcpSocket* socket);
    void HandleSocketDisconnected(QTcpSocket* socket);
    void ProcessBuffer(QTcpSocket* socket, bool flushPartialMessage);
    void HandleMessage(const QByteArray& message, const QString& peerDescription);

    static bool TryParseSnapshot(const QByteArray& message, SLAM::MapSnapshot* snapshot, QString* errorMessage);

    QTcpServer server_;
    quint16 port_ = 0;
    QHash<QTcpSocket*, QByteArray> buffers_;
    std::function<void(const SLAM::MapSnapshot&)> snapshotHandler_;
    std::function<void(const QString&)> statusHandler_;
};
