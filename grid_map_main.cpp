/*
 * Based on
 * PolygonIteratorTest.cpp
 *
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include <grid_map_core/grid_map_core.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_ros/GridMapRosConverter.hpp"

#include "grid_map_cv/grid_map_cv.hpp"

#include <sensor_msgs/image_encodings.h>


// Eigen
#include <Eigen/Core>

// Limits
#include <cfloat>

// Vector
#include <vector>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace Eigen;
using namespace grid_map;

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


int main ( int argc, char **argv )
{
  // constants
  std::string image_route="./../Images/";
  std::string image_file="cor_map_05_00_new1.pgm";

  if (argc==3){
     image_route = argv[1];
     image_file = argv[2];
  }
  //debug
  std::cout<< "using file ["<< image_route + image_file <<"]\n";

  // grid size in pixels
  double num_rows;
  double num_cols;
  //! map resolution pixel/meter
  double resolution=1.0;
  //2d position of the grid map in the grid map frame [m].
  // 0,0 puts the grid centered around 0,0
  double orig_x=0;
  double orig_y=0;

  //! global frame id (for maps)
  std::string global_frame="world";

  // map layer name for this test
  std::string layer_name="layer_name";

  // cell values
  double  minValue;
  double  maxValue;

  // load an image from cv
  std::cout<< "Loading image"  <<"\n";
  cv::Mat imageCV = cv::imread((image_route+image_file), CV_LOAD_IMAGE_UNCHANGED );
  num_rows = imageCV.rows;
  num_cols = imageCV.cols;

  cv::minMaxLoc(imageCV, &minValue, &maxValue);

  std::cout<< "Image size [" << num_rows <<", " << num_cols <<"]\n";
  std::cout<< "Min, max values [" << minValue <<", " << maxValue <<"]\n";
  std::cout<< "Channels [" <<imageCV.channels() <<"]\n";
  std::cout<<"Encoding  [" << type2str(imageCV.type())<<"]\n";

  // create empty grid map
  std::cout<< "Creating empty grid map"  <<"\n";
  grid_map::GridMap  map_(vector<string>({std::string(layer_name)}));
  map_.setGeometry(Length(num_rows, num_cols), resolution, Position(orig_x, orig_y));
  map_.setFrameId(global_frame);
  map_.clearAll();

  // Convert to grid map.
  std::cout<< "Converting to ros msg"  <<"\n";
  sensor_msgs::ImagePtr imageROS = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageCV).toImageMsg();

  std::cout<< "Converting ros msg to gridmap"  <<"\n";
  GridMapRosConverter::addLayerFromImage(*imageROS, layer_name, map_);

  /*
  Direct conversion cv-grid fails miserably...
  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(imageCV, "layer_name", map_, minValue, maxValue);
  */

  // do nasty things on it...
  std::cout<< "Drawing a circle"  <<"\n";
  Position center(0.0, 0.0); // meters
   Length length(10.0, 70.0);
  double radius = 50;  // meters


  // do nasty things on it...
/*  for (grid_map::CircleIterator iterator(map_, center, radius);
      !iterator.isPastEnd(); ++iterator)  {
    map_.at(layer_name, *iterator) = 0.5;
  }
*/
  for (grid_map::EllipseIterator iterator(map_, center, length, M_PI_4);
      !iterator.isPastEnd(); ++iterator)  {
    map_.at(layer_name, *iterator) = 0.5;
  }


  // Convert to image.
  std::cout<< "Reconverting into ros msg"  <<"\n";

  sensor_msgs::Image imageROSout;


  GridMapRosConverter::toImage(map_, layer_name, "mono8", imageROSout);

  std::cout<< "Reconverting into cv"  <<"\n";
  cv_bridge::CvImagePtr imageCVout;
  imageCVout=cv_bridge::toCvCopy(imageROSout, "mono8");

  std::cout<< "And saving"  <<"\n";
  cv::imwrite( image_route + image_file+"2.png", imageCVout->image );

  return 0;
}
