#pragma once

#ifndef SOCKETCOMM_H
#define SOCKETCOMM_H

#include <QObject>
#include <QMutex>
#include <QString>
#include <QThread>

#include <hal/Socket.h>

#include <atomic>
#include <memory>
#include <vector>

/**
 * Socket communication mode selected by ui->scoketComb:
 *   "TCP Server" | "TCP Client" | "UDP"
 */
enum class SocketCommMode
{
    TcpClient,
    TcpServer,
    Udp
};

/**
 * Background worker for TCP Client / TCP Server / UDP.
 *
 * - TCP Client: recv/send on an already-connected socket
 * - TCP Server: accept() on a listening socket, then recv/send on the client
 * - UDP:       recvfrom/sendto (peer address updated on receive)
 *
 * Runs on a dedicated QThread; call startReceiving via QThread::started.
 * Cross-thread sendMsg is queued onto the worker thread.
 */
class SocketWorker : public QObject
{
    Q_OBJECT

public:
    explicit SocketWorker(QObject* parent = nullptr);
    ~SocketWorker() override;

    /// TCP Client: connected socket (open + connect already done).
    void setSocket(std::unique_ptr<rl::hal::Socket> socket, std::size_t bufferSize = 4096);

    /// TCP Server: listening socket (open + bind + listen already done).
    void setListenSocket(std::unique_ptr<rl::hal::Socket> listenSocket, std::size_t bufferSize = 4096);

    /// UDP: bound socket + initial peer address for sendto.
    void setUdpSocket(std::unique_ptr<rl::hal::Socket> socket,
                      const rl::hal::Socket::Address& peerAddress,
                      std::size_t bufferSize = 4096);

    SocketCommMode mode() const { return m_mode; }

    /// Thread-safe enough for UI→worker use: sends under m_sendMutex.
    /// (Worker thread is blocked in recv, so QueuedConnection cannot be used.)
    void sendMsg(const QString& data);

public slots:
    void startReceiving();
    void stopReceiving();

signals:
    void dataReceived(const QString& data);
    void errorOccurred(const QString& msg);
    void disconnected();
    /// TCP Server: a client has been accepted (peer host string).
    void clientAccepted(const QString& peerInfo);
    /// Informational status (e.g. "listening...").
    void statusMessage(const QString& msg);

private slots:
    void doSend(const QString& data);

private:
    void runTcpClientLoop();
    void runTcpServerLoop();
    void runUdpLoop();

    SocketCommMode                   m_mode{ SocketCommMode::TcpClient };
    std::unique_ptr<rl::hal::Socket> m_socket;       ///< active I/O socket (client / accepted / UDP)
    std::unique_ptr<rl::hal::Socket> m_listenSocket; ///< TCP Server listen socket
    rl::hal::Socket::Address         m_peerAddress;  ///< UDP peer (updated by recvfrom)
    bool                             m_hasPeer{ false };
    std::atomic<bool>                m_running{ false };
    std::size_t                      m_bufferSize{ 4096 };
    QMutex                           m_sendMutex;
};

#endif // SOCKETCOMM_H
