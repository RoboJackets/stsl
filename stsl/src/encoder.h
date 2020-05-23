#ifndef SOFTWARETRAININGSUPPORTLIBRARY_ENCODER_H
#define SOFTWARETRAININGSUPPORTLIBRARY_ENCODER_H

#include <atomic>
#include <thread>
#include <gpiod.hpp>

// TODO Add a way to zero the position?

class Encoder {
public:
  Encoder(const std::string& line_name);

  ~Encoder();

  int getPosition();

private:
    gpiod::line line_;
    std::atomic_bool interrupted_{false};
    std::atomic_int  position_{0};
    std::thread encoder_thread_;

    void encoderWatcherFunction();

};


#endif //SOFTWARETRAININGSUPPORTLIBRARY_ENCODER_H
