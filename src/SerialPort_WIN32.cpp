#include <iostream>
#include "STSL/SerialPort_WIN32.h"

using namespace std;

SerialPort_WIN32::SerialPort_WIN32() {

}

SerialPort_WIN32::~SerialPort_WIN32() {
    Close();
}

bool SerialPort_WIN32::Open(std::string device, unsigned long baud) {

    port_handle_ = CreateFile(device.c_str(),GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

    SetParameters(baud);

    return (port_handle_ != INVALID_HANDLE_VALUE);
}

void SerialPort_WIN32::Close() {
    if(port_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(port_handle_);
        port_handle_ = INVALID_HANDLE_VALUE;
    }
}

void SerialPort_WIN32::Write(std::string message) {
    if(!WriteFile(port_handle_, message.c_str(), static_cast<DWORD>(message.size()), nullptr, nullptr)) {
//      TODO Error handling
        cerr << "Write failed" << endl;
    }
}

std::string SerialPort_WIN32::ReadLine() {
    std::string line;
    DWORD bytesRead = 0;
    uint8_t in = 0;
    do {
        if(!ReadFile(port_handle_, &in, 1, &bytesRead, nullptr)) {
//            TODO Error handling
            cerr << "Read failed" << endl;
            break;
        }
        if(in != '\n') {
            line += static_cast<char>(in);
        }
    } while(in != '\n');
    return line;
}

void SerialPort_WIN32::SetParameters(unsigned long baud ) {
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
//    TODO Error handling
    if (!GetCommState(port_handle_, &dcbSerialParams)) {
        cerr << "could not get the state of the comport" << endl;
    }
    dcbSerialParams.BaudRate=baud;
    dcbSerialParams.ByteSize=8;
    dcbSerialParams.StopBits=ONESTOPBIT;
    dcbSerialParams.Parity=NOPARITY;
//    TODO Error handling
    if(!SetCommState(port_handle_, &dcbSerialParams)){
        cerr << "analyse error" << endl;
    }
}
