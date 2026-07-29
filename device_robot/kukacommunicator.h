#ifndef KUKACOMMUNICATOR_H
#define KUKACOMMUNICATOR_H

#include <QObject>
#include <QString>
#include <QByteArray>
#include <QMetaType>
#include <QAbstractSocket>
#include <array>

class QTcpSocket;

// 机器人回传的实时数据结构（E2 + 6 关节 + 6 笛卡尔位姿）
struct RobotData
{
    double joint[7] = { 0, 0, 0, 0, 0, 0, 0 };   // [0]=E2, [1..6]=A1~A6 (deg)
    double pose[6] = { 0, 0, 0, 0, 0, 0 };      // X,Y,Z (mm) / A,B,C (deg)
};
Q_DECLARE_METATYPE(RobotData)

/**
 * KUKA EthernetKRL 通信类。
 * 架构：上位机作为 TCP 客户端主动连接，KUKA 机器人作为服务端监听。
 * （对应 EKI 配置 XML 中 <EXTERNAL><TYPE>Client</TYPE>，即外部系统为客户端）
 *
 * 运行在独立线程的事件循环中，通过信号槽与 UI 透传收发数据。
 *
 * 用法：
 *   auto *comm = new KukaCommunicator;      // 不要指定 parent
 *   auto *thread = new QThread;
 *   comm->moveToThread(thread);
 *   // ip 为机器人控制器地址（如 172.31.1.147），port 为 EKI 监听端口
 *   connect(thread, &QThread::started, comm, [comm]{ comm->start("172.31.1.147", 59152); });
 *   thread->start();
 */
class KukaCommunicator : public QObject
{
    Q_OBJECT
public:
    explicit KukaCommunicator(QObject* parent = nullptr);
    ~KukaCommunicator() override;

public slots:
    // 主动连接机器人（在通信线程内调用）。ip 为机器人地址，port 为 EKI 服务端口
    void start(const QString& ip, quint16 port);
    // 关闭 TCP（不发 IsOut；用于内部重建连接）
    void stop();
    // 优雅断开：先发 IsOut=TRUE，再 disconnectFromHost（供 UI/关机使用）
    void gracefulStop(const std::array<double, 7>& xp2 = {});

    // 设置后续报文中的 PTP 速度百分比 (1~100)，默认 30
    void setPtpVelocity(double percent);

    // 单步控制（velocityPercent 为空则使用 setPtpVelocity 的值）
    void stepMove(int stepMode, const std::array<double, 7>& xp2);
    void stepMove(int stepMode, const std::array<double, 7>& xp2, double velocityPercent);

    // 复位 (StepMode=0, IsOut=FALSE, 位姿用 XP2)
    void returnHome(const std::array<double, 7>& xp2);

    // 断开/退出 (IsOut=TRUE, 位姿用 XP2)
    void sendDisconnect(const std::array<double, 7>& xp2);

    // 直接发送原始报文（备用）
    void sendRaw(const QString& xml);

signals:
    // 解析后的机器人数据，透传给 UI
    void robotDataReceived(const RobotData& data);
    // 状态/日志消息
    void statusMessage(const QString& msg);
    // 与机器人的连接状态变化
    void clientConnected(bool connected);
    // 连接断开
    void clientDisconnected();

private slots:
    void onConnected();
    void onReadyRead();
    void onSocketDisconnected();
    void onSocketError(QAbstractSocket::SocketError error);

private:
    // 构造发送给机器人的 Sensor 报文（含 Velocity PTP%）
    static QString buildSensorXml(bool isOut, int stepMode, double velocity,
        double se1, double sx, double sy, double sz,
        double sa, double sb, double sc);
    static double clampVelocity(double percent);
    // 从机器人报文中提取单个标签内容
    static bool extractTag(const QString& src, const QString& tag, QString& out);
    // 解析机器人回传报文为 RobotData
    static bool parseRobotData(const QString& str, RobotData& data);
    // 实际发送到已连接的机器人
    void writeToSocket(const QString& str);

private:
    QTcpSocket* m_socket = nullptr;   // 主动连向机器人的套接字
    bool        m_connecting = false;
    QByteArray  m_recvBuffer;         // 粘包/分包处理缓冲
    double      m_ptpVelocity = 30.0; // PTP 速度百分比，写入 Sensor/Velocity
};

#endif // KUKACOMMUNICATOR_H