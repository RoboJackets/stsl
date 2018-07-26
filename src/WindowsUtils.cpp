#include <iostream>
#include <windows.h>
#include "STSL/WindowsUtils.h"

using namespace std;

std::string WindowsUtils::FindRobot() {
    auto com_ports = AvailableComPorts();
    if(!com_ports.empty())
        return com_ports[0];
    return std::string{};
}

std::vector<std::string> WindowsUtils::AvailableComPorts() {

    HKEY registry_key = nullptr;
    if(RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"), 0, KEY_QUERY_VALUE, &registry_key) != ERROR_SUCCESS) {
        cerr << "Unable to open serial COM enumeration registry key." << endl;
        return {};
    }

    DWORD cValues;

    long retCode = RegQueryInfoKey(registry_key, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &cValues, nullptr, nullptr, nullptr, nullptr);

    if(retCode != ERROR_SUCCESS) {
        cerr << "Unable to read info from registry key." << endl;
        return {};
    }

    vector<string> comPorts(cValues);

    if (cValues > 0)
    {
        constexpr size_t MAX_VALUE_NAME = 16383;
        TCHAR  achValue[MAX_VALUE_NAME];
        DWORD cchValue;
        for (DWORD i=0; i<cValues; i++)
        {
            cchValue = MAX_VALUE_NAME;
            achValue[0] = '\0';
            unsigned char data_buffer[6];
            std::fill(begin(data_buffer), end(data_buffer), '\0');
            DWORD data_buffer_size = 6;
            retCode = RegEnumValue(registry_key, i, achValue, &cchValue, nullptr, nullptr, data_buffer, &data_buffer_size);

            if (retCode == ERROR_SUCCESS )
            {
                comPorts[i] = string(data_buffer, data_buffer + 6);
            }
        }
    }

    RegCloseKey(registry_key);

    return comPorts;
}
