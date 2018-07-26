#ifndef TRAININGSUPPORTLIBRARY_OSXUTILS_H
#define TRAININGSUPPORTLIBRARY_OSXUTILS_H

#include "OSUtils.h"
#include <vector>

class OSXUtils : public OSUtils {
public:
    virtual std::string FindRobot() override;

protected:

    std::vector<std::string> FindConnectedArduinos();

};


#endif //TRAININGSUPPORTLIBRARY_OSXUTILS_H
