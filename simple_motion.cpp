// Linux version of the simple motion example (header-only driver)

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <chrono>
#include <thread>

// Minimal Linux serial class compatible with the driver API
class LinuxSerial
{
public:
    LinuxSerial(const char *device) : fd_(-1)
    {
        fd_ = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            perror("open serial port");
        }
    }
    ~LinuxSerial()
    {
        if (fd_ >= 0)
            close(fd_);
    }
    bool begin(long /*baudrate*/)
    {
        if (fd_ < 0)
            return false;
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd_, &tty) != 0)
            return false;
        cfsetospeed(&tty, B1000000);
        cfsetispeed(&tty, B1000000);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2; // 0.2s read timeout
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            return false;
        return true;
    }
    void setTimeout(int ms) { timeout_ = ms; }
    int write(const void *buf, size_t len) { return ::write(fd_, buf, len); }
    int write(uint8_t b) { return ::write(fd_, &b, 1); }
    int read()
    {
        uint8_t b;
        int n = ::read(fd_, &b, 1);
        return n == 1 ? b : -1;
    }
    size_t readBytes(uint8_t *buf, size_t len)
    {
        size_t total = 0;
        auto start = std::chrono::steady_clock::now();
        while (total < len)
        {
            int n = ::read(fd_, buf + total, len - total);
            if (n > 0)
            {
                total += n;
            }
            else
            {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        return total;
    }
    operator bool() const { return fd_ >= 0; }

private:
    int fd_;
    int timeout_ = 2;
};

// Make the driver use LinuxSerial as HardwareSerial
#define HardwareSerial LinuxSerial
#include "STSServoDriver.hpp"

int main()
{
    const char *serial_dev = "/dev/ttyUSB0";
    LinuxSerial serial(serial_dev);
    if (!serial.begin(1000000))
    {
        std::cerr << "Failed to open serial port at 1Mbps: " << serial_dev << std::endl;
        return 1;
    }
    serial.setTimeout(2);

    STSServoDriver servos;
    const uint8_t SERVO_ID = 1; // change to your servo's ID

    if (!servos.init(&serial))
    {
        std::cerr << "Servo init failed" << std::endl;
        return 1;
    }

    servos.setMode(0xFE, STSMode::POSITION);

    // Simple motion: 0 -> 2048 -> 4095
    auto wait_until_done = [&](uint8_t id)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while (servos.isMoving(id))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    };

    if (!servos.ping(SERVO_ID))
    {
        std::cerr << "Servo " << int(SERVO_ID) << " not responding" << std::endl;
    }

    servos.setTargetPosition(SERVO_ID, 0);
    wait_until_done(SERVO_ID);

    servos.setTargetPosition(SERVO_ID, 2048);
    wait_until_done(SERVO_ID);

    servos.setTargetPosition(SERVO_ID, 4095, 500);
    wait_until_done(SERVO_ID);

    return 0;
}