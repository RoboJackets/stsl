#ifdef __WIN32

#ifndef TRAININGSUPPORTLIBRARY_WINDOWSUTILS_H
#define TRAININGSUPPORTLIBRARY_WINDOWSUTILS_H

#include "OSUtils.h"
#include <vector>

class WindowsUtils : public OSUtils {
public:
    virtual std::string FindRobot() override;

protected:

    std::vector<std::string> AvailableComPorts();
};


#endif //TRAININGSUPPORTLIBRARY_WINDOWSUTILS_H

#endif