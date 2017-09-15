#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "STSL/SerialPort_OSX.h"

using namespace std;

SerialPort_OSX::~SerialPort_OSX() {
    Close();
}

bool SerialPort_OSX::Open(std::string device, unsigned int baud) {
    cout << device.c_str() << endl;
    port_handle_ = open(device.c_str(), O_RDWR | O_NOCTTY);

    SetProperties(baud);

    return (port_handle_ != -1);
}

void SerialPort_OSX::Close() {
    if(port_handle_ != -1) {
        close(port_handle_);
    }
}

void SerialPort_OSX::Write(std::string message) {
    ssize_t n_written = 0;
    ssize_t spot = 0;
    const char *msg_data = message.c_str();
    do {
        n_written = write( port_handle_, &msg_data[spot], message.size() - n_written );
        if(n_written < 0) {
            std::cerr << "Serial Write Error: " << strerror(errno) << std::endl;
            return;
        }
        spot += n_written;
    } while (spot < message.size() && n_written > 0);
}

std::string SerialPort_OSX::ReadLine() {
    string line = "";
    ssize_t n_read = 0;
    char in;
    do {
        n_read = read(port_handle_, &in, 1);
        if(n_read < 0) {
            std::cerr << strerror(errno) << std::endl;
            break;
        }
        if(n_read > 0 && in != '\n')
            line += in;
    } while(in != '\n' && n_read > 0);
    return line;
}

void SerialPort_OSX::SetProperties(unsigned int baud) {
    struct termios options;
    memset (&options, 0, sizeof(options));

/* Error Handling */
    if ( tcgetattr ( port_handle_, &options ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

/* Set Baud Rate */
    cfsetospeed (&options, baud);
    cfsetispeed (&options, baud);

/* Setting other Port Stuff */
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

/* Flush Port, then applies attributes */
//    tcflush( port_handle_, TCIFLUSH );
    if ( tcsetattr ( port_handle_, TCSAFLUSH, &options ) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
}
