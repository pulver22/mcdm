/*
 * Testbench for grid_map manipulation
 *
 */

#include "RFIDGridmap.h"


int main ( int argc, char **argv )
{
  // default values
  std::string image_route="./../Images/";
  std::string image_file="cor_map_05_00_new1.pgm";

  // grid pararameters
  double mapResolution = 1;
  double rfidMapResolution = 1;

  // ellipse information
  double likelihood= 10.0;
  double antennaX= 100.0;
  double antennaY= 100.0;
  double antennaHeading= 0;//
  // ellipse axes are derived from these two.
  double minX= -2.5;
  double maxX= 27.0;

  // shitloads of prints
  bool debug = true;

  if (argc==3){
     image_route = argv[1];
     image_file = argv[2];
  }

  RFIDGridmap myGrid((image_route+image_file),  mapResolution,  rfidMapResolution,  debug);


  // do nasty things on it...
  likelihood = 1.0;
  // myGrid.addEllipse(likelihood, 0,  100,  0, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 25,  100,  M_PI/4, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 50,  100,  M_PI/2, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 75,  100,  3*M_PI/4, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 100,  100,  M_PI, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 125,  100,  5*M_PI/4, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 150,  100,  3*M_PI/2, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 175,  100,  7*M_PI/4, -0.5, 7.0);
  //
  // myGrid.addEllipse(likelihood, 50,  50,  2*M_PI, -0.5, 7.0);
  // // one pixel thick line
  // myGrid.addLine(likelihood, 0.0, 0.0, 100, 100);

 // huge circle placed bottom left
// myGrid.addCircle(likelihood, 0.0, 0.0, 20);
 // huge circle placed middle of X axis == vertical
 myGrid.addCircle(likelihood, 20.0, 0.0, 20);
 // huge circle placed  middle of y axis == horizontal
// myGrid.addCircle(likelihood, 0.0, 100.0, 60);
 // huge circle placed  center of image
// myGrid.addCircle(likelihood, 100.0, 100.0, 80);
  //
  // myGrid.getCell(50,50);
  //
  // myGrid.setCell(100, 100, 0);
  //
  // myGrid.getPosition(50, 50);
  //
  // myGrid.setPosition(0, 100, 0);


  myGrid.saveAs( (image_route + image_file+"2.png") );

  return 0;
}
