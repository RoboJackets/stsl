#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include <boost/asio.hpp>
#include "OSUtils.h"

class RJRobot {

public:
    RJRobot();

    ~RJRobot();

private:

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;

    std::unique_ptr<OSUtils> os_utils_;

};


#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
