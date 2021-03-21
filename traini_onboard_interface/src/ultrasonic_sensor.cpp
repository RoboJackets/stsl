#include "ultrasonic_sensor.h"

using namespace std::chrono_literals;

UltrasonicSensor::UltrasonicSensor(const std::string &trigger_line_name,
                                   const std::string &echo_line_name,
                                   UltrasonicSensor::Callback_t measurement_callback)
    : measurement_callback_(measurement_callback)
{
    trigger_line_ = gpiod::find_line(trigger_line_name);
    if(!trigger_line_)
    {
        throw std::logic_error("Could not find line with name: " + trigger_line_name);
    }
    trigger_line_.request({"robot_interface_node", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
    if(!trigger_line_.is_requested())
    {
        throw std::logic_error("Could not take ownership of " + trigger_line_name);
    }

    echo_line_ = gpiod::find_line(echo_line_name);
    if(!echo_line_)
    {
        throw std::logic_error("Could not find line with name: " + echo_line_name);
    }
    echo_line_.request({"robot_interface_node", gpiod::line_request::EVENT_BOTH_EDGES, 0});
    if(!echo_line_.is_requested())
    {
        throw std::logic_error("Could not take ownership of " + echo_line_name);
    }

    echo_listener_thread_ = std::thread{[this](){ echoListenerThreadFunction(); }};
}

UltrasonicSensor::~UltrasonicSensor() {
    interrupted_ = true;
    echo_listener_thread_.join();
}

void UltrasonicSensor::trigger() {

    trigger_line_.set_value(1);
    std::this_thread::sleep_for(10us);
    trigger_line_.set_value(0);
}

void UltrasonicSensor::echoListenerThreadFunction() {
    std::chrono::nanoseconds rise_time;
    while(!interrupted_)
    {
        if(echo_line_.event_wait(std::chrono::seconds(1)))
        {
            const auto& event = echo_line_.event_read();
            if(event.event_type == gpiod::line_event::RISING_EDGE)
            {
                rise_time = event.timestamp;
            }
            else
            {
                const auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(event.timestamp - rise_time).count();
                measurement_callback_(elapsed_time * 170.0f);
            }
        }
    }
}
