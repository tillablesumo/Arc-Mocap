#include "udpreceiver.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QDateTime>

UdpReceiver::UdpReceiver(QObject *parent) : QObject(parent), m_udpSocket(nullptr)
{
    m_udpSocket = new QUdpSocket(this);
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &UdpReceiver::processPendingDatagrams);
}

UdpReceiver::~UdpReceiver()
{
    stopListening();
    delete m_udpSocket;
}

bool UdpReceiver::startListening(uint16_t port)
{
    if (m_udpSocket->isOpen()) {
        m_udpSocket->close();
    }

    if (!m_udpSocket->bind(port, QUdpSocket::ShareAddress)) {
        return false;
    }

    return true;
}

void UdpReceiver::stopListening()
{
    if (m_udpSocket->isOpen()) {
        m_udpSocket->close();
    }
}

void UdpReceiver::processPendingDatagrams()
{
    while (m_udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(m_udpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        m_udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        // 解析JSON数据
        QJsonParseError parseError;
        QJsonDocument jsonDoc = QJsonDocument::fromJson(datagram, &parseError);

        if (parseError.error != QJsonParseError::NoError) {
            emit logReceived("[ERROR] Failed to parse JSON: " + parseError.errorString());
            continue;
        }

        QJsonObject rootObject = jsonDoc.object();

        // 检查是否是VR控制器日志
        if (rootObject.value("node_type").toString() == "vr_controller") {
            uint8_t controllerId = rootObject.value("controller_id").toInt();
            QString logType = rootObject.value("log_type").toString();

            if (logType == "joystick") {
                // 解析摇杆数据
                QJsonObject joystickObj = rootObject.value("joystick").toObject();
                float x = joystickObj.value("x").toDouble();
                float y = joystickObj.value("y").toDouble();

                // 发送信号
                emit joystickDataReceived(controllerId, x, y);

                // 生成日志
                QString controllerName = (controllerId == 0) ? "左手柄" : "右手柄";
                QString log = QString("[%1] %2 - 摇杆：X=%3, Y=%4").arg(
                    QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"),
                    controllerName,
                    QString::number(x, 'f', 2),
                    QString::number(y, 'f', 2)
                );
                emit logReceived(log);
            } else if (logType == "button") {
                // 解析按键数据
                QJsonObject buttonsObj = rootObject.value("buttons").toObject();
                bool trigger = buttonsObj.value("trigger").toBool();
                bool grip = buttonsObj.value("grip").toBool();
                bool a = buttonsObj.value("a").toBool();
                bool x = buttonsObj.value("x").toBool();
                bool b = buttonsObj.value("b").toBool();
                bool y = buttonsObj.value("y").toBool();
                bool menu = buttonsObj.value("menu").toBool();
                bool joystickClick = buttonsObj.value("joystick_click").toBool();

                // 发送信号
                emit buttonDataReceived(controllerId, trigger, grip, a, x, b, y, menu, joystickClick);

                // 生成日志
                QString controllerName = (controllerId == 0) ? "左手柄" : "右手柄";
                QString buttonStates;
                if (trigger) buttonStates += "Trigger ";
                if (grip) buttonStates += "Grip ";
                if (a) buttonStates += "A ";
                if (x) buttonStates += "X ";
                if (b) buttonStates += "B ";
                if (y) buttonStates += "Y ";
                if (menu) buttonStates += "Menu ";
                if (joystickClick) buttonStates += "Joystick Click ";

                if (!buttonStates.isEmpty()) {
                    buttonStates.chop(1); // 移除最后一个空格
                    QString log = QString("[%1] %2 - 按键：%3 %4").arg(
                        QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"),
                        controllerName,
                        buttonStates,
                        (trigger || grip || a || x || b || y || menu || joystickClick) ? "按下" : "松开"
                    );
                    emit logReceived(log);
                }
            }
        }
    }
}
