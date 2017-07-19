#include "STSL/LinuxUtils.h"
#include <libudev.h>
#include <iostream>
#include <unistd.h>

using namespace std;

std::string LinuxUtils::FindRobot() {
    return FindConnectedArduinos().front();
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

        auto devpath = udev_device_get_devnode(device);

        device = udev_device_get_parent_with_subsystem_devtype(
                device,
                "usb",
                "usb_device");

        if(device == nullptr) {
            continue;
        }

        auto idVendor = string{udev_device_get_sysattr_value(device, "idVendor")};
        auto idProduct = string{udev_device_get_sysattr_value(device, "idProduct")};

        if(idVendor == "2a03" && idProduct == "0043") {
            // Found Aruino UNO
            arduinos.push_back(string{devpath});
        } else if(idVendor == "0403" && idProduct == "6001") {
            // Found Arduino Duemilanove
            arduinos.push_back(string{devpath});
        }

    }

    udev_enumerate_unref(enumerator);

    udev_unref(udev);

    return arduinos;
}

void LinuxUtils::Sleep(std::chrono::microseconds duration) {
    usleep(static_cast<__useconds_t>(duration.count()));
}
