#ifdef __linux__

#include "STSL/LinuxUtils.h"
#include <libudev.h>
#include <iostream>

using namespace std;

std::string LinuxUtils::FindRobot() {
    auto arduinos = FindConnectedArduinos();

    // TODO for each arduino, open the port, listen for signal, flag as robot or not, clsoe the port
    // TODO return first arduino which responds as expected

    return std::__cxx11::string();
}

std::vector<std::string> LinuxUtils::FindConnectedArduinos() {

    udev *udev;
    udev_enumerate *enumerator;
    udev_list_entry *devices;
    udev_list_entry *dev_list_entry;
    udev_device *device;

    udev = udev_new();
    if(udev == nullptr) {
        cerr << "Can't create udev handle." << endl;
        return {};
    }

    enumerator = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerator, "tty");
    udev_enumerate_scan_devices(enumerator);
    devices = udev_enumerate_get_list_entry(enumerator);

    vector<string> arduinos;

    udev_list_entry_foreach(dev_list_entry, devices) {
        auto path = udev_list_entry_get_name(dev_list_entry);
        device = udev_device_new_from_syspath(udev, path);

        device = udev_device_get_parent_with_subsystem_devtype(
                device,
                "usb",
                "usb_device");

        if(device == nullptr) {
            continue;
        }

        auto idVendor = string{udev_device_get_sysattr_value(device, "idVendor")};
        auto idProduct = string{udev_device_get_sysattr_value(device, "idProduct")};

        // Check for Arduino UNO VID and PID
        if(idVendor == "0043" && idProduct == "2341") {
            arduinos.push_back(string{path});
        }

    }

    udev_enumerate_unref(enumerator);

    udev_unref(udev);

    return arduinos;
}

#endif