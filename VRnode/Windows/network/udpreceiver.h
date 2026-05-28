#pragma once

#include <QObject>
#include <QUdpSocket>

class UdpReceiver : public QObject
{
    Q_OBJECT

public:
    explicit UdpReceiver(QObject *parent = nullptr);
    ~UdpReceiver() override;

    bool startListening(uint16_t port);
    void stopListening();

signals:
    void joystickDataReceived(uint8_t controllerId, float x, float y);
    void buttonDataReceived(uint8_t controllerId, bool trigger, bool grip, bool a, bool x, bool b, bool y, bool menu, bool joystickClick);
    void logReceived(const QString &log);

private slots:
    void processPendingDatagrams();

private:
    QUdpSocket *m_udpSocket;
};
