// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "src/encoder.hpp"
#include <string>

using namespace std::chrono_literals;

Encoder::Encoder(const std::string & line_name)
{
  line_ = gpiod::find_line(line_name);
  if (!line_) {
    throw std::logic_error("Could not find line with name: " + line_name);
  }
  line_.request({"robot_interface_node", gpiod::line_request::EVENT_BOTH_EDGES, 0});
  if (!line_.is_requested()) {
    throw std::logic_error("Could not take ownership of " + line_name);
  }
  encoder_thread_ = std::thread([this]() {encoderWatcherFunction();});
}

Encoder::~Encoder()
{
  interrupted_ = true;
  encoder_thread_.join();
}

int Encoder::getPosition()
{
  return position_;
}

void Encoder::encoderWatcherFunction()
{
  while (!interrupted_) {
    if (line_.event_wait(1s)) {
      line_.event_read();
      position_++;
    }
  }
}
