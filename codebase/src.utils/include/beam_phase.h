#ifndef _BEAM_PHASE_H
#define _BEAM_PHASE_H

double  beamdirs_rad[16]={      -.4241, -.3676, -.3110, -.2545, -.1979, -.1414, -.0848, -.0283, 
                          .0283,  .0848,  .1414,  .1979,  .2545,  .3110,  .3676,  .4241};
double calculate_delta(double frequency, double beam_angle_rad, double antenna_spacing); 

#endif

