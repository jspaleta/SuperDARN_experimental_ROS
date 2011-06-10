#include "math.h"

double calculate_delta(double frequency, double beam_angle_rad, double antenna_spacing) {
  double pi=M_PI;
  double c= 299792458;
  double delta;
  double phase;

  delta=-(antenna_spacing*2.0*pi*sin(beam_angle_rad)*frequency)/c;
  return delta;
}

