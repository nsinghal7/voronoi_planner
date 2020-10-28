#pragma once

#include <string>
#include "car/car_param_parser.h"
#include <cmath>
#include <math.h>
#define PI M_PI

namespace car {

typedef struct {
  double start_x;
  double start_y;
  double start_theta;
  double start_v;
  double start_delta;

  double goal_x;
  double goal_y;
} FormationSpec;

FormationSpec generateFormationSpec(std::string formation, int car_num, int n_cars, CarParamParser& params) {
  if(formation == "fixed_circle_opposite") {
    double angle = 2 * PI * car_num / n_cars;
    double radius = 8;
    FormationSpec spec = {
      .start_x = cos(angle) * radius,
      .start_y = sin(angle) * radius,
      .start_theta = fmod(angle + PI / 2, 2*PI),
      .start_v = 0,
      .start_delta = 0,
      
      .goal_x = -cos(angle) * radius,
      .goal_y = -sin(angle) * radius,
    };
    return spec;
  } else if(formation == "sized_circle_opposite") {
    double angle = 2 * PI * car_num / n_cars;
    double radius = n_cars * 2;
    FormationSpec spec = {
      .start_x = cos(angle) * radius,
      .start_y = sin(angle) * radius,
      .start_theta = fmod(angle + PI / 2, 2*PI),
      .start_v = 0,
      .start_delta = 0,

      .goal_x = -cos(angle) * radius,
      .goal_y = -sin(angle) * radius,
    };
    return spec;
  } else {
    std::cerr << "Got unknown formation type: " << formation << std::endl;
    throw "unknown formation type";
  }
}

} // namespace car

