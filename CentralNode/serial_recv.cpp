#include "mocap_data.hpp"

#include <iostream>
#include <iomanip>
#include <windows.h>

class SerialReceiver {
private:
    HANDLE hSerial;
    bool is_open;

public:
    SerialReceiver() : hSerial(INVALID_HANDLE_VALUE), is_open(false) {}
    ~SerialReceiver() { close(); }

    bool init(const char* port_name, uint32_t baud_rate = 115200) {
        DCB dcbSerialParams = {0};
        COMMTIMEOUTS timeouts = {0};

        hSerial = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Failed to open serial port: " << port_name << std::endl;
            return false;
        }

        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Failed to get comm state" << std::endl;
            return false;
        }

        dcbSerialParams.BaudRate = baud_rate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Failed to set comm state" << std::endl;
            return false;
        }

        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "Failed to set timeouts" << std::endl;
            return false;
        }

        is_open = true;
        std::cout << "Serial port opened: " << port_name << std::endl;
        return true;
    }

    bool receive(MocapPacket& packet) {
        if (!is_open) return false;

        uint8_t buffer[sizeof(MocapPacket)];
        DWORD bytes_read = 0;

        if (!ReadFile(hSerial, buffer, sizeof(MocapPacket), &bytes_read, NULL)) {
            return false;
        }

        if (bytes_read != sizeof(MocapPacket)) {
            return false;
        }

        memcpy(&packet, buffer, sizeof(MocapPacket));
        return true;
    }

    void close() {
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
        is_open = false;
    }

    bool isConnected() const { return is_open; }
};

int main(int argc, char* argv[]) {
    const char* port_name = (argc > 1) ? argv[1] : "\\\\.\\COM7";

    std::cout << "=== MotionSnap Central Node (BLE to Serial) ===" << std::endl;
    std::cout << "Receiving data from BLE and forwarding to serial port: " << port_name << std::endl;

    SerialReceiver receiver;
    if (!receiver.init(port_name, 115200)) {
        std::cerr << "Failed to initialize serial port" << std::endl;
        return 1;
    }

    std::cout << "Waiting for BLE data..." << std::endl;

    while (true) {
        MocapPacket packet;
        if (receiver.receive(packet)) {
            std::cout << "Node:" << (int)packet.node_id
                      << " Seq:" << (int)packet.seq
                      << " Gyro:[" << packet.gyro_x << "," << packet.gyro_y << "," << packet.gyro_z << "]"
                      << " Accel:[" << packet.accel_x << "," << packet.accel_y << "," << packet.accel_z << "]"
                      << std::endl;
        }
    }

    return 0;
}