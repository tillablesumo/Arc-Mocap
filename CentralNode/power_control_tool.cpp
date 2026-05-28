#include <iostream>
#include <windows.h>

class SerialComm {
private:
    HANDLE hSerial;
    bool is_open;

public:
    SerialComm() : hSerial(INVALID_HANDLE_VALUE), is_open(false) {}
    ~SerialComm() { close(); }

    bool open(const char* port_name, uint32_t baud_rate = 115200) {
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

    bool send_power_command(uint8_t node_id, uint8_t power_level) {
        if (!is_open) return false;

        uint8_t command[3] = {'P', node_id, power_level};
        DWORD bytes_written = 0;

        if (!WriteFile(hSerial, command, sizeof(command), &bytes_written, NULL)) {
            std::cerr << "Failed to send power command" << std::endl;
            return false;
        }

        // 等待确认
        uint8_t response[3];
        DWORD bytes_read = 0;
        if (!ReadFile(hSerial, response, sizeof(response), &bytes_read, NULL)) {
            std::cerr << "Failed to read response" << std::endl;
            return false;
        }

        if (bytes_read == 3 && response[0] == 'A' && response[1] == node_id && response[2] == power_level) {
            return true;
        }

        return false;
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

void print_power_level_info() {
    std::cout << "=== BLE Transmit Power Levels ===" << std::endl;
    std::cout << "Level | Power (dBm)" << std::endl;
    std::cout << "------|-------------" << std::endl;
    std::cout << "0     | -12 dBm" << std::endl;
    std::cout << "1     |  -9 dBm" << std::endl;
    std::cout << "2     |  -6 dBm" << std::endl;
    std::cout << "3     |  -3 dBm" << std::endl;
    std::cout << "4     |   0 dBm" << std::endl;
    std::cout << "5     |   3 dBm" << std::endl;
    std::cout << "6     |   6 dBm" << std::endl;
    std::cout << "7     |   9 dBm" << std::endl;
    std::cout << "================================" << std::endl;
}

void print_menu() {
    std::cout << "\n=== Power Control Menu ===" << std::endl;
    std::cout << "1. Set power level for VRnode (Node ID: 1)" << std::endl;
    std::cout << "2. Set power level for VRController (Node ID: 2)" << std::endl;
    std::cout << "3. Set power level for VHMD (Node ID: 16)" << std::endl;
    std::cout << "4. Set power level for custom node ID" << std::endl;
    std::cout << "5. Exit" << std::endl;
    std::cout << "Enter your choice: ";
}

int main(int argc, char* argv[]) {
    const char* port_name = (argc > 1) ? argv[1] : "\\\\.\\COM7";

    SerialComm comm;
    if (!comm.open(port_name)) {
        return 1;
    }

    print_power_level_info();

    while (true) {
        print_menu();
        int choice;
        std::cin >> choice;

        if (choice == 5) {
            break;
        }

        uint8_t node_id;
        uint8_t power_level;

        switch (choice) {
            case 1:
                node_id = 1;
                break;
            case 2:
                node_id = 2;
                break;
            case 3:
                node_id = 16;
                break;
            case 4:
                std::cout << "Enter node ID (0-15): ";
                std::cin >> node_id;
                break;
            default:
                std::cout << "Invalid choice" << std::endl;
                continue;
        }

        std::cout << "Enter power level (0-7): ";
        std::cin >> power_level;

        if (power_level < 0 || power_level > 7) {
            std::cout << "Invalid power level. Must be 0-7." << std::endl;
            continue;
        }

        if (comm.send_power_command(node_id, power_level)) {
            std::cout << "Successfully set power level for node " << (int)node_id << " to " << (int)power_level << std::endl;
        } else {
            std::cout << "Failed to set power level" << std::endl;
        }
    }

    comm.close();
    std::cout << "Serial port closed. Exiting..." << std::endl;

    return 0;
}