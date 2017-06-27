#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <sstream>
#include <iostream>
#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  void print()
  {
    switch (sensor_type_)
    {
    case LASER:
      std::cout << "L" << "," << timestamp_ << "," << raw_measurements_[0] << "," << raw_measurements_[1] << std::endl;
      break;
    case RADAR:
      std::cout << "R" << "," << timestamp_ << "," << raw_measurements_[0] << "," << raw_measurements_[1] << "," << raw_measurements_[2] << std::endl;
      break;
    }
  }
};

#endif /* MEASUREMENT_PACKAGE_H_ */
