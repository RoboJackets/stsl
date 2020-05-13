#include "ultrasonic_sensor.h"

using namespace std::chrono_literals;

UltrasonicSensor::UltrasonicSensor(const std::string &trigger_line_name,
                                   const std::string &echo_line_name) {
    trigger_line_ = gpiod::find_line(trigger_line_name);
    if(!trigger_line_)
    {
        throw std::logic_error("Could not find line with name: " + trigger_line_name);
    }
    trigger_line_.request({"robot_interface_node", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);

    echo_line_ = gpiod::find_line(echo_line_name);
    if(!echo_line_)
    {
        throw std::logic_error("Could not find line with name: " + echo_line_name);
    }
    echo_line_.request({"robot_interface_node", gpiod::line_request::EVENT_BOTH_EDGES, 0});
}

float UltrasonicSensor::getDistance() {

    trigger_line_.set_value(1);
    std::this_thread::sleep_for(10us);
    trigger_line_.set_value(0);
    if(!echo_line_.event_wait(1ms))
    {
        return  -1.f;
    }
    const auto& rise_event = echo_line_.event_read();
    const auto& rise_time = rise_event.timestamp;
    if(!echo_line_.event_wait(1ms))
    {
        return -1.f;
    }
    const auto& fall_event = echo_line_.event_read();
    const auto& fall_time = fall_event.timestamp;

    const auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<float>>(fall_time - rise_time).count();

    return elapsed_time * 170.0f;
}
