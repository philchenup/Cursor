// =============================================================================
// MainWindow::InitializeComm — supports TCP Server / TCP Client / UDP
// (ui->scoketComb items: "TCP Server", "TCP Client", "UDP")
//
// Required MainWindow members (replace old m_udpSocket / m_udpMode if present):
//
//   #include "SocketComm.h"
//
//   SocketWorker* m_worker_comm = nullptr;
//   QThread*      m_thread_comm = nullptr;
//   bool          m_comm_connected = false;
//   void          setConnected(bool connected);  // updates UI + m_comm_connected
//
// Notes:
//   - All three modes run SocketWorker on m_thread_comm.
//   - TCP Server binds to ip (or 0.0.0.0 if empty) : port, listens, accepts.
//   - TCP Client connects to ip:port.
//   - UDP binds 0.0.0.0:port locally and sendto/recvfrom peer ip:port.
// =============================================================================

#include "mainwindow.h"
#include "SocketComm.h"

#include <QMessageBox>
#include <memory>
#include <stdexcept>

void MainWindow::InitializeComm()
{
    auto cleanupComm = [this]() {
        if (m_worker_comm)
            m_worker_comm->stopReceiving();

        if (m_thread_comm)
        {
            m_thread_comm->quit();
            m_thread_comm->wait(3000);
            // Worker is deleted via QThread::finished → deleteLater
            m_thread_comm->deleteLater();
            m_thread_comm = nullptr;
            m_worker_comm = nullptr;
        }

        setConnected(false);
    };

    connect(ui->connectBtn, &QPushButton::clicked, this, [=]() {
        if (m_comm_connected)
        {
            cleanupComm();
            return;
        }

        // ── connect ──
        const QString modeText = ui->scoketComb->currentText().trimmed();
        const QString ip = ui->ipaddressEdit->text().trimmed();
        const QString portStr = ui->commPortEdit->text().trimmed();

        const bool isTcpServer = (modeText == QStringLiteral("TCP Server"));
        const bool isUdp = (modeText == QStringLiteral("UDP"));
        // anything else (including "TCP Client") → TCP Client

        if (portStr.isEmpty())
        {
            ui->console->print(ct::LOG_WARNING, "Input Error, Please enter Port.");
            return;
        }

        if (!isTcpServer && ip.isEmpty())
        {
            ui->console->print(ct::LOG_WARNING, "Input Error, Please enter IP and Port.");
            return;
        }

        try
        {
            m_thread_comm = new QThread(this);
            m_worker_comm = new SocketWorker(); // no parent; owned by thread via deleteLater

            if (isUdp)
            {
                // Bind locally on the given port; peer = remote ip:port for sendto.
                const auto localAddr = rl::hal::Socket::Address::Ipv4(
                    std::string("0.0.0.0"), portStr.toStdString());
                const auto peerAddr = rl::hal::Socket::Address::Ipv4(
                    ip.toStdString(), portStr.toStdString());

                auto sock = std::make_unique<rl::hal::Socket>(
                    rl::hal::Socket::Udp(localAddr));
                sock->open();
                sock->bind();
                m_worker_comm->setUdpSocket(std::move(sock), peerAddr);
            }
            else if (isTcpServer)
            {
                const QString bindIp = ip.isEmpty() ? QStringLiteral("0.0.0.0") : ip;
                const auto address = rl::hal::Socket::Address::Ipv4(
                    bindIp.toStdString(), portStr.toStdString());

                auto sock = std::make_unique<rl::hal::Socket>(
                    rl::hal::Socket::Tcp(address));
                sock->open();
                sock->bind();
                sock->listen();
                m_worker_comm->setListenSocket(std::move(sock));
            }
            else
            {
                // TCP Client
                const auto address = rl::hal::Socket::Address::Ipv4(
                    ip.toStdString(), portStr.toStdString());

                auto sock = std::make_unique<rl::hal::Socket>(
                    rl::hal::Socket::Tcp(address));
                sock->open();
                sock->connect();
                m_worker_comm->setSocket(std::move(sock));
            }

            m_worker_comm->moveToThread(m_thread_comm);

            connect(m_worker_comm, &SocketWorker::dataReceived, this, [=](const QString& data) {
                ui->receiveEdit->setText(data);
            });

            connect(m_worker_comm, &SocketWorker::errorOccurred, this, [=](const QString& msg) {
                ui->console->print(ct::LOG_ERROR, msg);
            });

            connect(m_worker_comm, &SocketWorker::statusMessage, this, [=](const QString& msg) {
                ui->console->print(ct::LOG_INFO, msg);
            });

            connect(m_worker_comm, &SocketWorker::clientAccepted, this, [=](const QString& peer) {
                ui->console->print(ct::LOG_INFO,
                    QStringLiteral("TCP Server accepted client: %1").arg(peer));
            });

            connect(m_worker_comm, &SocketWorker::disconnected, this, [=]() {
                // Peer closed (TCP Client). Tear down the same way as Disconnect.
                if (m_worker_comm)
                    m_worker_comm->stopReceiving();

                if (m_thread_comm)
                {
                    m_thread_comm->quit();
                    m_thread_comm->wait(2000);
                    m_thread_comm->deleteLater();
                    m_thread_comm = nullptr;
                    m_worker_comm = nullptr;
                }
                setConnected(false);
            });

            connect(m_thread_comm, &QThread::started, m_worker_comm, &SocketWorker::startReceiving);
            connect(m_thread_comm, &QThread::finished, m_worker_comm, &QObject::deleteLater);

            m_thread_comm->start();
            setConnected(true);

            const char* readyMsg =
                isUdp ? "UDP connected."
                      : (isTcpServer ? "TCP Server listening."
                                     : "TCP Client connected.");
            ui->console->print(ct::LOG_INFO, readyMsg);
        }
        catch (const std::exception& e)
        {
            if (m_worker_comm)
            {
                delete m_worker_comm;
                m_worker_comm = nullptr;
            }
            if (m_thread_comm)
            {
                delete m_thread_comm;
                m_thread_comm = nullptr;
            }
            QMessageBox::critical(this, "Connection Failed", QString::fromStdString(e.what()));
        }
    });

    auto sendCurrent = [this]() {
        if (!m_comm_connected || !m_worker_comm)
            return;

        const QString msg = ui->sendEdit->text();
        if (msg.isEmpty())
            return;

        m_worker_comm->sendMsg(msg);
    };

    connect(ui->commSendBtn, &QPushButton::clicked, this, [=]() { sendCurrent(); });
    connect(ui->sendEdit, &QLineEdit::returnPressed, this, [=]() { sendCurrent(); });

    ui->console->print(ct::LOG_INFO, "Socket Communication is ready now!");
}
