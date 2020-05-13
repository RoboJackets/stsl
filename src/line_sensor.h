#ifndef SOFTWARETRAININGSUPPORTLIBRARY_LINE_SENSOR_H
#define SOFTWARETRAININGSUPPORTLIBRARY_LINE_SENSOR_H

#include <fstream>

class LineSensor {
public:
  LineSensor(int input_number);

  int getValue();

private:
  std::ifstream file;
};


#endif //SOFTWARETRAININGSUPPORTLIBRARY_LINE_SENSOR_H
