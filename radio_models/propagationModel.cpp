#include <cmath>
#include <iostream>


// constants ..................................................................
// We mostly use UPM frog 3D.
const double TAG_LOSSES = -4.8;

// 20 * log10 ( c / (4*pi) )
const double LOSS_CONSTANT = 147.55;
// 4*pi/c
const double PHASE_CONSTANT = 4.192e-8;

// This comes from the manufacturer. Azimut
// gain list entries start at -165 degrees to 180 in steps of 15.
const double antennaLossesList [24] = {  -25.2, -25, -20.2, -17.6, -15.6, -14, -11.2, -7.8, -5.2, -2.4, -0.6, 0, -0.6, -2.4, -5.2, -8.8, -12.2, -16.4, -19.2, -20.8, -24.4, -28.2, -24, -22.6};

// Minimum sensitivy
const double MIN_SENSITIVITY = -115.0;



//  ..................................................................


/**
 * returns MT_242025NRHK antenna losses in a plane
 * @param  angleRad spheric coordinate within the plane
 * @return          dB losses
 */
double antennaPlaneLoss(double angleRad ){
  double ans = -20.0;
  double angleDeg = angleRad*180.0/M_PI;

  // gain list entries start at -165 degrees to 180 in steps of 15.
  int index = ( ( (int) angleDeg ) + 165 ) / 15;

  if ( (index<24) & (index>=0) )
    ans = antennaLossesList[index];

  return ans;
}

float sign(float x){
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return 0.0;
}

/**
 * Returns Spherical coordinates
 * @param x          cartesian coordinate x
 * @param y          cartesian coordinate y
 * @param r          spheric coordinate r
 * @param phi        relative azimut [0,2pi) (angle between XY projection and X)
 */
void getSphericCoords(double x, double y, double& r, double& phi){
  r = sqrt(x*x+y*y);
  phi = atan2(y,x);

}

/**
 * Get received power from an OMNIDIRECTIONAL tag,
 * given its relative position to antenna.
 * We assume antenna at 0,0,0, facing X coordinate.
 * See http://www.antenna-theory.com/basics/friis.php
 * Sensitivity is -85 dBm / -115 dB
 *
 * @param  tag_x       Tag x coord with respect to antenna
 * @param  tag_y       Tag y coord with respect to antenna
 * @param  freq        Transmission frequency (Hertzs)
 * @param  txtPower    Transmitted power (dBs)
 * @return             Received power (dBs)
 */
double received_power_friis(double tag_x, double tag_y, double freq, double txtPower) {
    double phi;
    double r;

    getSphericCoords(tag_x,tag_y, r, phi);

    /*
     SIMPLIFICATION!!! TAG is OMNIDIRECTIONAL
     (a.k.a. don't have tag radiation pattern and
     Here they say it's ok https://www.hindawi.com/journals/ijap/2013/194145/tab4/
    */
    double antL =  TAG_LOSSES + antennaPlaneLoss(phi);


    // propagation losses
    double propL = LOSS_CONSTANT - (20 * log10  (r * freq)) ;

    // signal goes from antenna to tag and comes back again, so we double the losses
    double rxPower = txtPower + 2*antL + 2*propL ;
    return rxPower;
}

double phaseDifference(double tag_x, double tag_y, double freq) {
  double phi;
  double r;

  getSphericCoords(tag_x,tag_y, r, phi);
  double phase = PHASE_CONSTANT * freq * r;
  phase = fmod(phase, M_PI);
  return phase;

}
