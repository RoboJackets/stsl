#include <iostream>
#include <windows.h>
#include "STSL/WindowsUtils.h"

using namespace std;

std::string WindowsUtils::FindRobot() {
    auto com_ports = AvailableComPorts();
    return std::string{};
}

std::vector<std::string> WindowsUtils::AvailableComPorts() {

    // TODO attempting to use registry keys to enumerate available COM ports. This currently causes a mystery segfault

    HKEY registry_key = nullptr;
    if(RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"), 0, KEY_QUERY_VALUE, &registry_key) != ERROR_SUCCESS) {
        cerr << "Unable to open serial COM enumeration registry key." << endl;
        return {};
    }
//    unsigned char value[128];
//    DWORD value_length = 128;
//    RegQueryValueEx(registry_key, "value_name", nullptr, reinterpret_cast<LPDWORD>(REG_SZ), static_cast<LPBYTE>(value), &value_length);
//    cout << "Value = " << value << endl;
    RegCloseKey(registry_key);
}
