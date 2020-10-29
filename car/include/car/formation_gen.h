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

FormationSpec generateFormationSpec(CarParamParser& params) {
  auto formation = params.formation;
  int n_cars = params.n_cars;
  int car_num = params.car_num;
  if(formation == "fixed_circle_opposite") {
    double angle = 2 * PI * car_num / n_cars;
    double radius = 8;
    FormationSpec spec = {
      .start_x = cos(angle) * radius,
      .start_y = sin(angle) * radius,
      .start_theta = fmod(angle + PI / 2, 2*PI),
      .start_v = 0,
      .start_delta = 0,
      
      .goal_x = cos(angle + PI) * radius,
      .goal_y = sin(angle + PI) * radius,
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

      .goal_x = cos(angle + PI) * radius,
      .goal_y = sin(angle + PI) * radius,
    };
    return spec;
  } else {
    std::cerr << "Got unknown formation type: " << formation << std::endl;
    throw "unknown formation type";
  }
}

} // namespace car

