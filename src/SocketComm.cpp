#include "SocketComm.h"

#include <cstring>
#include <stdexcept>

SocketWorker::SocketWorker(QObject* parent)
    : QObject(parent)
{
}

SocketWorker::~SocketWorker()
{
    stopReceiving();
}

void SocketWorker::setSocket(std::unique_ptr<rl::hal::Socket> socket, std::size_t bufferSize)
{
    m_mode = SocketCommMode::TcpClient;
    m_listenSocket.reset();
    m_socket = std::move(socket);
    m_bufferSize = bufferSize;
    m_hasPeer = false;
}

void SocketWorker::setListenSocket(std::unique_ptr<rl::hal::Socket> listenSocket, std::size_t bufferSize)
{
    m_mode = SocketCommMode::TcpServer;
    m_socket.reset();
    m_listenSocket = std::move(listenSocket);
    m_bufferSize = bufferSize;
    m_hasPeer = false;
}

void SocketWorker::setUdpSocket(std::unique_ptr<rl::hal::Socket> socket,
                                const rl::hal::Socket::Address& peerAddress,
                                std::size_t bufferSize)
{
    m_mode = SocketCommMode::Udp;
    m_listenSocket.reset();
    m_socket = std::move(socket);
    m_peerAddress = peerAddress;
    m_hasPeer = true;
    m_bufferSize = bufferSize;
}

void SocketWorker::sendMsg(const QString& data)
{
    // The worker thread is blocked in accept/recv/recvfrom, so it cannot
    // process queued slots. Send directly under m_sendMutex (same pattern
    // as the original SocketWorker).
    if (m_running)
        doSend(data);
}

void SocketWorker::startReceiving()
{
    switch (m_mode)
    {
    case SocketCommMode::TcpServer:
        runTcpServerLoop();
        break;
    case SocketCommMode::Udp:
        runUdpLoop();
        break;
    case SocketCommMode::TcpClient:
    default:
        runTcpClientLoop();
        break;
    }
}

void SocketWorker::runTcpClientLoop()
{
    if (!m_socket)
    {
        emit errorOccurred(QStringLiteral("Worker: socket is null"));
        return;
    }

    m_running = true;
    std::vector<char> buffer(m_bufferSize);

    while (m_running)
    {
        try
        {
            std::memset(buffer.data(), 0, buffer.size());
            const std::size_t n = m_socket->recv(buffer.data(), buffer.size());

            if (n == 0)
            {
                emit disconnected();
                break;
            }

            emit dataReceived(QString::fromUtf8(buffer.data(), static_cast<int>(n)));
        }
        catch (const std::exception& e)
        {
            if (m_running)
                emit errorOccurred(QString::fromStdString(e.what()));
            break;
        }
    }

    m_running = false;
}

void SocketWorker::runTcpServerLoop()
{
    if (!m_listenSocket)
    {
        emit errorOccurred(QStringLiteral("Worker: listen socket is null"));
        return;
    }

    m_running = true;
    emit statusMessage(QStringLiteral("TCP Server listening, waiting for client..."));

    std::vector<char> buffer(m_bufferSize);

    while (m_running)
    {
        try
        {
            // Blocking accept; stopReceiving() closes the listen socket to unblock.
            // C++17 mandatory elision: accept() prvalue constructs heap Socket in place.
            m_socket.reset(new rl::hal::Socket(m_listenSocket->accept()));

            QString peer = QStringLiteral("unknown");
            try
            {
                peer = QString::fromStdString(m_socket->getAddress().getNameInfo(true));
            }
            catch (...)
            {
            }
            emit clientAccepted(peer);
            emit statusMessage(QStringLiteral("Client connected: %1").arg(peer));

            // Receive from this client until disconnect or stop.
            while (m_running && m_socket)
            {
                try
                {
                    std::memset(buffer.data(), 0, buffer.size());
                    const std::size_t n = m_socket->recv(buffer.data(), buffer.size());

                    if (n == 0)
                    {
                        emit statusMessage(QStringLiteral("Client disconnected, waiting for next client..."));
                        m_socket.reset();
                        break; // back to accept()
                    }

                    emit dataReceived(QString::fromUtf8(buffer.data(), static_cast<int>(n)));
                }
                catch (const std::exception& e)
                {
                    if (m_running)
                        emit errorOccurred(QString::fromStdString(e.what()));
                    m_socket.reset();
                    break; // back to accept(), or exit if !m_running
                }
            }
        }
        catch (const std::exception& e)
        {
            if (m_running)
                emit errorOccurred(QString::fromStdString(e.what()));
            break;
        }
    }

    m_running = false;
    m_socket.reset();
}

void SocketWorker::runUdpLoop()
{
    if (!m_socket)
    {
        emit errorOccurred(QStringLiteral("Worker: UDP socket is null"));
        return;
    }

    m_running = true;
    emit statusMessage(QStringLiteral("UDP ready"));

    std::vector<char> buffer(m_bufferSize);

    while (m_running)
    {
        try
        {
            std::memset(buffer.data(), 0, buffer.size());
            rl::hal::Socket::Address from;
            const std::size_t n = m_socket->recvfrom(buffer.data(), buffer.size(), from);

            if (n == 0)
                continue;

            {
                QMutexLocker locker(&m_sendMutex);
                m_peerAddress = from;
                m_hasPeer = true;
            }

            emit dataReceived(QString::fromUtf8(buffer.data(), static_cast<int>(n)));
        }
        catch (const std::exception& e)
        {
            if (m_running)
                emit errorOccurred(QString::fromStdString(e.what()));
            break;
        }
    }

    m_running = false;
}

void SocketWorker::stopReceiving()
{
    m_running = false;

    // Closing sockets unblocks accept/recv/recvfrom.
    if (m_socket)
    {
        try { m_socket->close(); }
        catch (...) {}
        m_socket.reset();
    }

    if (m_listenSocket)
    {
        try { m_listenSocket->close(); }
        catch (...) {}
        m_listenSocket.reset();
    }
}

void SocketWorker::doSend(const QString& data)
{
    QMutexLocker locker(&m_sendMutex);

    if (!m_running)
    {
        emit errorOccurred(QStringLiteral("Send failed: not connected"));
        return;
    }

    const QByteArray bytes = data.toUtf8();

    try
    {
        if (m_mode == SocketCommMode::Udp)
        {
            if (!m_socket || !m_hasPeer)
            {
                emit errorOccurred(QStringLiteral("Send failed: no UDP peer"));
                return;
            }
            m_socket->sendto(bytes.constData(), static_cast<std::size_t>(bytes.size()), m_peerAddress);
        }
        else
        {
            // TCP Client or TCP Server (accepted client socket)
            if (!m_socket)
            {
                emit errorOccurred(m_mode == SocketCommMode::TcpServer
                    ? QStringLiteral("Send failed: no client connected yet")
                    : QStringLiteral("Send failed: not connected"));
                return;
            }
            m_socket->send(bytes.constData(), static_cast<std::size_t>(bytes.size()));
        }
    }
    catch (const std::exception& e)
    {
        emit errorOccurred(QStringLiteral("Send error: ") + e.what());
    }
}
