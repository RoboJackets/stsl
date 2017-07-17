#ifdef __linux__

#ifndef TRAININGSUPPORTLIBRARY_LINUXUTILS_H
#define TRAININGSUPPORTLIBRARY_LINUXUTILS_H

#include "OSUtils.h"
#include <vector>

class LinuxUtils : public OSUtils {
public:
    virtual std::string FindRobot() override;

protected:

    std::vector<std::string> FindConnectedArduinos();

};


#endif //TRAININGSUPPORTLIBRARY_LINUXUTILS_H

#endif