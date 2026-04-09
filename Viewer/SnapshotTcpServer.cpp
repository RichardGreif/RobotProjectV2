#include "SnapshotTcpServer.h"

#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTcpSocket>

namespace
{
    bool ReadPoint(const QJsonObject& object, const char* key, Vec2* point, QString* errorMessage)
    {
        const QJsonValue value = object.value(key);
        if (!value.isObject())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QString("Missing object '%1'.").arg(key);
            }
            return false;
        }

        const QJsonObject pointObject = value.toObject();
        const QJsonValue xValue = pointObject.value("x");
        const QJsonValue yValue = pointObject.value("y");
        if (!xValue.isDouble() || !yValue.isDouble())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QString("Point '%1' must contain numeric x and y values.").arg(key);
            }
            return false;
        }

        point->x = static_cast<float>(xValue.toDouble());
        point->y = static_cast<float>(yValue.toDouble());
        return true;
    }
}

SnapshotTcpServer::SnapshotTcpServer(quint16 port, QObject* parent)
    : QObject(parent),
      port_(port)
{
    server_.setParent(this);
    server_.setListenBacklogSize(4);
    server_.setMaxPendingConnections(4);
}

bool SnapshotTcpServer::Start(QString* errorMessage)
{
    if (!server_.listen(QHostAddress::Any, port_))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = server_.errorString();
        }
        return false;
    }

    QObject::connect(&server_, &QTcpServer::newConnection, this, [this]()
    {
        HandleNewConnection();
    });

    if (statusHandler_)
    {
        statusHandler_(QString("Listening for MapSnapshot TCP data on port %1").arg(server_.serverPort()));
    }

    return true;
}

quint16 SnapshotTcpServer::GetPort() const
{
    return server_.serverPort();
}

void SnapshotTcpServer::SetSnapshotHandler(const std::function<void(const SLAM::MapSnapshot&)>& handler)
{
    snapshotHandler_ = handler;
}

void SnapshotTcpServer::SetStatusHandler(const std::function<void(const QString&)>& handler)
{
    statusHandler_ = handler;
}

void SnapshotTcpServer::HandleNewConnection()
{
    while (server_.hasPendingConnections())
    {
        QTcpSocket* socket = server_.nextPendingConnection();
        if (socket == nullptr)
        {
            continue;
        }

        buffers_.insert(socket, QByteArray());

        QObject::connect(socket, &QTcpSocket::readyRead, this, [this, socket]()
        {
            HandleSocketReadyRead(socket);
        });

        QObject::connect(socket, &QTcpSocket::disconnected, this, [this, socket]()
        {
            HandleSocketDisconnected(socket);
        });

        if (statusHandler_)
        {
            statusHandler_(QString("Client connected: %1:%2")
                .arg(socket->peerAddress().toString())
                .arg(socket->peerPort()));
        }
    }
}

void SnapshotTcpServer::HandleSocketReadyRead(QTcpSocket* socket)
{
    if (socket == nullptr || !buffers_.contains(socket))
    {
        return;
    }

    buffers_[socket].append(socket->readAll());
    ProcessBuffer(socket, false);
}

void SnapshotTcpServer::HandleSocketDisconnected(QTcpSocket* socket)
{
    if (socket == nullptr)
    {
        return;
    }

    if (buffers_.contains(socket))
    {
        ProcessBuffer(socket, true);
        buffers_.remove(socket);
    }

    socket->deleteLater();
}

void SnapshotTcpServer::ProcessBuffer(QTcpSocket* socket, bool flushPartialMessage)
{
    QByteArray& buffer = buffers_[socket];

    while (true)
    {
        const int newlineIndex = buffer.indexOf('\n');
        if (newlineIndex < 0)
        {
            break;
        }

        const QByteArray message = buffer.left(newlineIndex).trimmed();
        buffer.remove(0, newlineIndex + 1);

        if (!message.isEmpty())
        {
            HandleMessage(message, QString("%1:%2").arg(socket->peerAddress().toString()).arg(socket->peerPort()));
        }
    }

    if (flushPartialMessage)
    {
        const QByteArray message = buffer.trimmed();
        buffer.clear();

        if (!message.isEmpty())
        {
            HandleMessage(message, QString("%1:%2").arg(socket->peerAddress().toString()).arg(socket->peerPort()));
        }
    }
}

void SnapshotTcpServer::HandleMessage(const QByteArray& message, const QString& peerDescription)
{
    SLAM::MapSnapshot snapshot;
    QString errorMessage;
    if (!TryParseSnapshot(message, &snapshot, &errorMessage))
    {
        if (statusHandler_)
        {
            statusHandler_(QString("Invalid snapshot from %1: %2").arg(peerDescription, errorMessage));
        }
        return;
    }

    if (snapshotHandler_)
    {
        snapshotHandler_(snapshot);
    }

    if (statusHandler_)
    {
        statusHandler_(QString("Loaded snapshot with %1 wall(s) from %2").arg(snapshot.wallCount).arg(peerDescription));
    }
}

bool SnapshotTcpServer::TryParseSnapshot(const QByteArray& message, SLAM::MapSnapshot* snapshot, QString* errorMessage)
{
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(message, &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = parseError.errorString();
        }
        return false;
    }

    if (!document.isObject())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = "Top-level JSON value must be an object.";
        }
        return false;
    }

    const QJsonObject root = document.object();
    const QJsonValue wallsValue = root.value("walls");
    if (!wallsValue.isArray())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = "Missing array 'walls'.";
        }
        return false;
    }

    const QJsonArray wallsArray = wallsValue.toArray();
    if (wallsArray.size() > SLAM::MapSnapshot::MaxWalls)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QString("Snapshot contains %1 walls, max is %2.")
                .arg(wallsArray.size())
                .arg(SLAM::MapSnapshot::MaxWalls);
        }
        return false;
    }

    snapshot->Clear();

    for (int i = 0; i < wallsArray.size(); ++i)
    {
        if (!wallsArray.at(i).isObject())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QString("Wall %1 must be an object.").arg(i);
            }
            return false;
        }

        const QJsonObject wallObject = wallsArray.at(i).toObject();
        SLAM::WallSegment segment;
        if (!ReadPoint(wallObject, "start", &segment.start, errorMessage))
        {
            return false;
        }

        if (!ReadPoint(wallObject, "end", &segment.end, errorMessage))
        {
            return false;
        }

        const QJsonValue confidenceValue = wallObject.value("confidence");
        if (!confidenceValue.isDouble())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QString("Wall %1 must contain numeric confidence.").arg(i);
            }
            return false;
        }

        segment.confidence = static_cast<float>(confidenceValue.toDouble());
        snapshot->walls[i] = segment;
        snapshot->wallCount = i + 1;
    }

    return true;
}
