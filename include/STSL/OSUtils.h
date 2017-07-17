#ifndef TRAININGSUPPORTLIBRARY_OSUTILS_H
#define TRAININGSUPPORTLIBRARY_OSUTILS_H

#include <string>

class OSUtils {
public:
    virtual std::string FindRobot() = 0;
};

#endif //TRAININGSUPPORTLIBRARY_OSUTILS_H
