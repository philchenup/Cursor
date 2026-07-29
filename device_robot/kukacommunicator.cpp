#include "kukacommunicator.h"
#include <QTcpSocket>
#include <QHostAddress>
#include <QAbstractSocket>
#include <QThread>

KukaCommunicator::KukaCommunicator(QObject* parent)
    : QObject(parent)
{
    // 跨线程队列连接必须注册元类型，否则 robotDataReceived 信号无法投递
    qRegisterMetaType<RobotData>("RobotData");
}

KukaCommunicator::~KukaCommunicator()
{
    stop();
}

void KukaCommunicator::start(const QString& ip, quint16 port)
{
    // 先彻底停掉旧连接，避免重复 start 时资源泄漏
    stop();

    if (ip.isEmpty()) {
        emit statusMessage(QStringLiteral("连接失败：机器人 IP 为空"));
        return;
    }

    QHostAddress addr;
    if (!addr.setAddress(ip)) {
        emit statusMessage(QStringLiteral("连接失败：非法 IP 地址 %1").arg(ip));
        return;
    }

    m_socket = new QTcpSocket(this);
    connect(m_socket, &QTcpSocket::connected,
        this, &KukaCommunicator::onConnected);
    connect(m_socket, &QTcpSocket::readyRead,
        this, &KukaCommunicator::onReadyRead);
    connect(m_socket, &QTcpSocket::disconnected,
        this, &KukaCommunicator::onSocketDisconnected);
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    connect(m_socket, &QTcpSocket::errorOccurred,
        this, &KukaCommunicator::onSocketError);
#else
    connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
        this, &KukaCommunicator::onSocketError);
#endif

    m_recvBuffer.clear();
    emit statusMessage(QStringLiteral("正在连接机器人 %1:%2 ...").arg(ip).arg(port));
    m_socket->connectToHost(addr, port);
}

void KukaCommunicator::stop()
{
    if (m_socket) {
        // 先断开信号，避免关闭过程中重入 onSocketDisconnected
        m_socket->disconnect(this);
        if (m_socket->state() == QAbstractSocket::ConnectedState) {
            // 优雅关 TCP；勿用 abort，否则示教器易报通信断开
            m_socket->disconnectFromHost();
            if (m_socket->state() != QAbstractSocket::UnconnectedState) {
                m_socket->waitForDisconnected(1000);
            }
        } else if (m_socket->state() != QAbstractSocket::UnconnectedState) {
            m_socket->abort();
        }
        m_socket->deleteLater();
        m_socket = nullptr;
    }

    m_recvBuffer.clear();
    if (m_connecting) {
        m_connecting = false;
        emit clientConnected(false);
    }
}

void KukaCommunicator::gracefulStop(const std::array<double, 7>& xp2)
{
    if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
        // 1) 协议层通知机器人：IsOut=TRUE → kukarec 内 EKI_Close
        sendDisconnect(xp2);
        m_socket->waitForBytesWritten(500);
        // 2) 给 Submit/pcservice / kukarec 处理 EKI_Close 的时间
        QThread::msleep(150);
    }
    // 3) 再关 TCP
    stop();
}

void KukaCommunicator::onConnected()
{
    if (!m_socket) {
        return;
    }

    m_recvBuffer.clear();
    emit statusMessage(QStringLiteral("已连接机器人：%1:%2")
        .arg(m_socket->peerAddress().toString())
        .arg(m_socket->peerPort()));
    m_connecting = true;
    emit clientConnected(true);
}

void KukaCommunicator::onReadyRead()
{
    if (!m_socket) {
        return;
    }

    m_recvBuffer.append(m_socket->readAll());

    // 直接在字节缓冲上按 </Robot> 切分完整报文，处理 TCP 粘包/分包
    static const QByteArray endTag = QByteArrayLiteral("</Robot>");

    int endIdx;
    while ((endIdx = m_recvBuffer.indexOf(endTag)) >= 0) {
        const int msgEnd = endIdx + endTag.size();
        const QString oneMsg = QString::fromLatin1(m_recvBuffer.constData(), msgEnd); // KUKA EKI 默认 ASCII
        m_recvBuffer.remove(0, msgEnd);

        RobotData data;
        if (parseRobotData(oneMsg, data)) {
            emit robotDataReceived(data);
        }
    }

    // 防护：若长期收不到结束标签（协议异常），避免缓冲无限增长
    if (m_recvBuffer.size() > 1 * 1024 * 1024) {
        emit statusMessage(QStringLiteral("接收缓冲异常增长，已清空"));
        m_recvBuffer.clear();
    }
}

void KukaCommunicator::onSocketDisconnected()
{
    emit statusMessage(QStringLiteral("与机器人断开连接"));
    emit clientDisconnected();

    if (m_socket) {
        m_socket->disconnect(this);
        m_socket->deleteLater();
        m_socket = nullptr;
    }
    m_recvBuffer.clear();
    m_connecting = false;
    emit clientConnected(false);
}

void KukaCommunicator::onSocketError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    if (!m_socket) {
        return;
    }
    // 已连接后的 RemoteHostClosedError 会再走 disconnected，这里只提示
    emit statusMessage(QStringLiteral("套接字错误：") + m_socket->errorString());
}

// —— 发送相关槽 ——
void KukaCommunicator::setPtpVelocity(double percent)
{
    m_ptpVelocity = clampVelocity(percent);
}

void KukaCommunicator::stepMove(int stepMode, const std::array<double, 7>& xp2)
{
    stepMove(stepMode, xp2, m_ptpVelocity);
}

void KukaCommunicator::stepMove(int stepMode, const std::array<double, 7>& xp2, double velocityPercent)
{
    writeToSocket(buildSensorXml(false, stepMode, clampVelocity(velocityPercent),
        xp2[0], xp2[1], xp2[2], xp2[3], xp2[4], xp2[5], xp2[6]));
}

void KukaCommunicator::returnHome(const std::array<double, 7>& xp2)
{
    writeToSocket(buildSensorXml(false, 0, m_ptpVelocity,
        xp2[0], xp2[1], xp2[2], xp2[3], xp2[4], xp2[5], xp2[6]));
}

void KukaCommunicator::sendDisconnect(const std::array<double, 7>& xp2)
{
    writeToSocket(buildSensorXml(true, 0, m_ptpVelocity,
        xp2[0], xp2[1], xp2[2], xp2[3], xp2[4], xp2[5], xp2[6]));
}

void KukaCommunicator::sendRaw(const QString& xml)
{
    writeToSocket(xml);
}

void KukaCommunicator::writeToSocket(const QString& str)
{
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        emit statusMessage(QStringLiteral("发送失败：机器人未连接"));
        return;
    }
    m_socket->write(str.toLatin1());
    m_socket->flush();
}

double KukaCommunicator::clampVelocity(double percent)
{
    if (percent < 1.0) {
        return 1.0;
    }
    if (percent > 100.0) {
        return 100.0;
    }
    return percent;
}

// —— 报文构造 ——
QString KukaCommunicator::buildSensorXml(bool isOut, int stepMode, double velocity,
    double se1, double sx, double sy, double sz,
    double sa, double sb, double sc)
{
    return QStringLiteral(
        "<Sensor><Status><IsOut>%1</IsOut></Status><StepMode>%2</StepMode>"
        "<Velocity>%3</Velocity>"
        "<SJ1>%4</SJ1><SJ2>%5</SJ2><SJ3>%6</SJ3><SJ4>%7</SJ4>"
        "<SJ5>%8</SJ5><SJ6>%9</SJ6><SE2>%10</SE2></Sensor>")
        .arg(isOut ? QStringLiteral("TRUE") : QStringLiteral("FALSE"))
        .arg(stepMode)
        .arg(velocity, 0, 'f', 2)
        .arg(sx, 0, 'f', 4).arg(sy, 0, 'f', 4).arg(sz, 0, 'f', 4)
        .arg(sa, 0, 'f', 4).arg(sb, 0, 'f', 4).arg(sc, 0, 'f', 4).arg(se1, 0, 'f', 4);
}

// —— 报文解析 ——
bool KukaCommunicator::extractTag(const QString& src, const QString& tag, QString& out)
{
    const QString open = QStringLiteral("<%1>").arg(tag);
    const QString close = QStringLiteral("</%1>").arg(tag);

    const int start = src.indexOf(open);
    if (start < 0) {
        return false;
    }
    const int contentStart = start + open.length();
    const int end = src.indexOf(close, contentStart);
    if (end < 0) {
        return false;
    }
    out = src.mid(contentStart, end - contentStart).trimmed();
    return true;
}

bool KukaCommunicator::parseRobotData(const QString& str, RobotData& data)
{
    static const QString jointTags[7] =
    { QStringLiteral("E2"), QStringLiteral("J1"), QStringLiteral("J2"),
      QStringLiteral("J3"), QStringLiteral("J4"), QStringLiteral("J5"),
      QStringLiteral("J6") };
    static const QString poseTags[6] =
    { QStringLiteral("PX"), QStringLiteral("PY"), QStringLiteral("PZ"),
      QStringLiteral("PA"), QStringLiteral("PB"), QStringLiteral("PC") };

    bool ok = true;
    QString val;

    for (int i = 0; i < 7; ++i) {
        if (extractTag(str, jointTags[i], val)) {
            bool conv = false;
            const double d = val.toDouble(&conv);
            if (conv) data.joint[i] = d; else ok = false;
        }
        else {
            ok = false;
        }
    }
    for (int i = 0; i < 6; ++i) {
        if (extractTag(str, poseTags[i], val)) {
            bool conv = false;
            const double d = val.toDouble(&conv);
            if (conv) data.pose[i] = d; else ok = false;
        }
        else {
            ok = false;
        }
    }
    return ok;
}