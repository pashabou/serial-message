#pragma once

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <sys/ioctl.h>

/**
 * Analogue of Arduino Serial object for non-Arduino communication
 * - supports begin, readBytes, write, available, flush
 */
struct SerialInterface {
private:
  /** Serial file descriptor */
  int serial;

public:
  /** Setup serial for macOS & Linux machines */
  void begin(const char* filename, long baudrate = B115200) {
    serial = open(filename, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial < 0) exit(1);
    
    termios tty;
    if (tcgetattr(serial, &tty) != 0) exit(2);

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_iflag &= ~IGNBRK;                 // no break processing
    tty.c_lflag = 0;                        // no signal characters
    tty.c_oflag = 0;                        // no remapping
    tty.c_cc[VMIN] = 0;                     // no blocking
    tty.c_cc[VTIME] = 0;                    // no read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no xon / xoff control
    tty.c_cflag |= CLOCAL | CREAD;          // ignore modem control
    tty.c_cflag &= ~CRTSCTS;                // no RTS/CTS flow control

    // data bits
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // parity bits
    tty.c_cflag &= ~(PARENB | PARODD);

    // stop bits
    tty.c_cflag &= ~CSTOPB;
    
    if (tcsetattr(serial, TCSANOW, &tty) != 0) exit(3);
  }

  /** Write to file descriptor */
  ssize_t write(char* buffer, int size) {
    return ::write(serial, buffer, size);
  }

  /** Read from file descriptor */
  ssize_t readBytes(char* buffer, int size) {
    return read(serial, buffer, size);
  }

  /** Get number of bytes in input buffer */
  int available() {
    int bytes;
    ioctl(serial, FIONREAD, &bytes);
    return bytes;
  }

  void clearInput() { tcflush(serial, TCIFLUSH); }
  void clearOutput() { tcflush(serial, TCOFLUSH); }
  void flush() {}
};

SerialInterface Serial;
