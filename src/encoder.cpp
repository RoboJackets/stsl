#include "encoder.h"

using namespace std::chrono_literals;

Encoder::Encoder(const std::string &line_name) {
    line_ = gpiod::find_line(line_name);
    if(!line_)
    {
        throw std::logic_error("Could not find line with name: " + line_name);
    }
    line_.request({"robot_interface_node", gpiod::line_request::EVENT_BOTH_EDGES, 0});
    encoder_thread_ = std::thread([this](){encoderWatcherFunction();});
}

Encoder::~Encoder() {
    interrupted_ = true;
    encoder_thread_.join();
}

int Encoder::getPosition() {
    return position_;
}

void Encoder::encoderWatcherFunction() {
    while(!interrupted_)
    {
        if(line_.event_wait(1s))
        {
            line_.event_read();
            position_++;
        }
    }
}
