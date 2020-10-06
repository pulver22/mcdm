#include "RadarModel.hpp"
#include <unsupported/Eigen/SpecialFunctions>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <omp.h>
#include <chrono>

using namespace std;
using namespace grid_map;

// http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt

//////////////////  SPLINE FUNCTIONS   //////////////////

// The spline is used to interpolate antenna gain values, as we only have the
// graphs
SplineFunction::SplineFunction() {}

SplineFunction::SplineFunction(Eigen::VectorXd const &x_vec,
                               Eigen::VectorXd const &y_vec)
    : x_min(x_vec.minCoeff()), x_max(x_vec.maxCoeff()), y_min(y_vec.minCoeff()),
      y_max(y_vec.maxCoeff()),
      // Spline fitting here. X values are scaled down to [0, 1] for this.
      spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
          y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 6),
          scaled_values(
              x_vec))) // No more than cubic spline, but accept short vectors.
{}

// x values need to be scaled down in extraction as well.
double SplineFunction::interpDeg(double x) const {
  double y;
  y = spline_(scaled_value(x))(0);

  // interpolation may produce values bigger and lower than our limits ...
  y = max(min(y, y_max), y_min);
  return y;
}

double SplineFunction::interpRad(double x) const {
  return interpDeg(x * 180.0 / M_PI);
}

float SplineFunction::interpRadf(float x) const {
    return (float) interpDeg(( (double) x) *180.0/M_PI); 
}

// Helpers to scale X values down to [0, 1]
double SplineFunction::scaled_value(double x) const {
  return (x - x_min) / (x_max - x_min);
}

Eigen::RowVectorXd
SplineFunction::scaled_values(Eigen::VectorXd const &x_vec) const {
  return x_vec.unaryExpr([this](double x) { return scaled_value(x); })
      .transpose();
}

//////////////////

RadarModel::RadarModel(){};

RadarModel::RadarModel(const double resolution, const double sigma_power, const double sigma_phase, const double txtPower, const std::vector<double> freqs, const std::vector<std::pair<double,double>> tags_coords, const std::string imageFileURI ) {
        _sigma_power = sigma_power;
        _sigma_phase = sigma_phase;
        _txtPower = txtPower;
        _freqs = freqs;
        _resolution = resolution;
        _tags_coords = tags_coords;
        _numTags =  tags_coords.size();

        initRefMap(imageFileURI);

        // build spline to interpolate antenna gains;
        std::vector<double> xVec(ANTENNA_ANGLES_LIST, ANTENNA_ANGLES_LIST + 25);
        Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xVec.data(), xVec.size());
        std::vector<double> yVec(ANTENNA_LOSSES_LIST, ANTENNA_LOSSES_LIST + 25);
        Eigen::VectorXd yvals= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yVec.data(), yVec.size());
        
        _antenna_gains= SplineFunction(xvals, yvals);
      
        // rfid beliefs global map: One layer per tag
        std::string layerName;
        for(int i = 0; i <_numTags; ++i) {
           layerName = getTagLayerName(i);
           _rfid_belief_maps.add(layerName, 0.5);  // the cells need to have a uniform distribution at the beginning
        }
        clearObstacleCellsRFIDMap();
        normalizeRFIDMap();
        debugInfo();

        // mesh grid layers
        _rfid_belief_maps.add("X", 0);  
        _rfid_belief_maps.add("Y", 0);  
        Position point;
        Index ind;
        for (grid_map::GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd(); ++iterator) {
            // matrix indexes...
            ind = *iterator;
            // get cell center of the cell in the map frame.            
            _rfid_belief_maps.getPosition(ind, point);
            // that is where the tag supposedly is in map coordinates
            _rfid_belief_maps.at("X",*iterator) = point.x();
            _rfid_belief_maps.at("Y",*iterator) = point.y();
        }

        // Tags have not been detected yet
        for(int i=0; i<tags_coords.size();i++){
          _first_detection.push_back(false);
          _iteration_no_readings.push_back(1);
        }
        _belief_tags.reserve(tags_coords.size());
        assert( tags_coords.size() == _first_detection.size());

        // Add tmp later for KL-divergence computation
        _rfid_belief_maps.add("kl", 0.0);

    }


void RadarModel::initRefMap(const std::string imageURI) {
  std::cout << "\nIniting Ref map." << std::endl;

  cv::Mat _imageCV = cv::imread(imageURI, CV_LOAD_IMAGE_UNCHANGED);
  // this alligns image with our coordinate systems
  cv::flip(_imageCV, _imageCV, -1);

  _Ncol = _imageCV.cols; // radar model total x-range space (cells).
  _Nrow = _imageCV.rows; // radar model total y-range space (cells).

  // cell value ranges
  double minValue;
  double maxValue;

  double orig_x, len_x, orig_y, len_y;

  len_x = _Nrow * _resolution;
  len_y = _Ncol * _resolution;
  // 2D position of the grid map in the grid map frame [m].
  // orig will be placed at the UPPER LEFT CORNER THE IMAGE, thus all
  // coordinates will be positive

  orig_x = (_Nrow - 1) * (_resolution / 2.0);
  orig_y = (_Ncol - 1) * (_resolution / 2.0);

  _rfid_belief_maps = grid_map::GridMap(vector<string>({"ref_map"}));
  _rfid_belief_maps.setGeometry(Length(len_x, len_y), _resolution,
                                Position(orig_x, orig_y));
  _tmp_rfid_c_map.setGeometry(Length(2, 2), _resolution);
  _tmp_rfid_c_map.add("temp", 0.0);

  cv::minMaxLoc(_imageCV, &minValue, &maxValue);
  _free_space_val = maxValue;
  _rfid_belief_maps["ref_map"].setConstant(NAN);
  GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
      _imageCV, "ref_map", _rfid_belief_maps, minValue, maxValue);

  std::cout << " Input map has " << _rfid_belief_maps.getSize()(1)
            << " cols by " << _rfid_belief_maps.getSize()(0) << " rows "
            << std::endl;
  std::cout << " Orig at: (" << orig_x << ", " << orig_y << ") m. "
            << std::endl;
  std::cout << " Size: (" << _rfid_belief_maps.getLength().x() << ", "
            << _rfid_belief_maps.getLength().y() << ") m. " << std::endl;
  std::cout << " Values range: (" << minValue << ", " << maxValue << ")   "
            << std::endl;
  std::cout << " Using : (" << maxValue << ") value as free space value  "
            << std::endl;

  grid_map::Position p;
  grid_map::Index index;

  // std::cout<<"\nTesting boundaries: " <<std::endl;
  index = grid_map::Index(0, 0);
  // std::cout<<"P: Index("  << 0 << ", " << 0 << ") " <<std::endl;
  if (_rfid_belief_maps.getPosition(index, p)) {
    // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" <<
    // p(0) << ", " << p(1)<<") m. " <<std::endl;
  } else {
    // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out
    // bounds!" <<std::endl;
  }

  index = grid_map::Index(_Nrow - 1, 0);
  // std::cout<<"P: Index("  << (_Nrow-1) << ", " << 0 << ") " <<std::endl;
  if (_rfid_belief_maps.getPosition(index, p)) {
    // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" <<
    // p(0) << ", " << p(1)<<") m. " <<std::endl;
  } else {
    // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out
    // bounds!" <<std::endl;
  }

  index = grid_map::Index(0, _Ncol - 1);
  // std::cout<<"P: Index("  << (0) << ", " << (_Ncol-1) << ") " <<std::endl;
  if (_rfid_belief_maps.getPosition(index, p)) {
    // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" <<
    // p(0) << ", " << p(1)<<") m. " <<std::endl;
  } else {
    // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out
    // bounds!" <<std::endl;
  }

  index = grid_map::Index(_Nrow - 1, _Ncol - 1);
  // std::cout << "P: Index("  << (_Nrow-1) << ", " << (_Ncol-1) << ") "
  // <<std::endl;
  if (_rfid_belief_maps.getPosition(index, p)) {
    // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" <<
    // p(0) << ", " << p(1)<<") m. " <<std::endl;
  } else {
    // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out
    // bounds!" <<std::endl;
  }
  // std::cout << "............................. " << std::endl << std::endl;
}

void RadarModel::saveProbMaps(std::string savePath) {
  std::string tagLayerName;
  std::string fileURI;
  Eigen::MatrixXf data_mat;

  // PrintMap(savePath);

  // prob distribution maps
  for (int i = 0; i < _numTags; ++i) {
    tagLayerName = getTagLayerName(i);
    // std::cout << " Saving layer [" << tagLayerName << "]" << std::endl ;
    fileURI = savePath + "final_prob_" + tagLayerName + ".png";
    getImage(&_rfid_belief_maps, tagLayerName, fileURI);
    // data_mat = _rfid_belief_maps[tagLayerName];
    // PrintProb(fileURI, &data_mat, _Ncol*_resolution, _Nrow*_resolution,
    // _resolution);
  }
}

//////////////////////////  GETTERS
///////////////////////////////////////////////////////
Eigen::MatrixXf  RadarModel::getPowProbCond(double rxPw,  double f_i){
  return getPowProbCond( rxPw,  0,  0,  0,  f_i);
}

Eigen::MatrixXf  RadarModel::getPowProbCond(double rxPw, double x_m, double y_m, double orientation_deg, double f_i){
  Eigen::MatrixXf PW_mat = getFriisMat(x_m,y_m,orientation_deg, f_i);
  Eigen::MatrixXf ans = getProbCond(PW_mat, rxPw, _sigma_power);
  return ans;
}


Eigen::MatrixXf  RadarModel::getPhaseProbCond(double rxPw,  double f_i){
  return getPhaseProbCond( rxPw,  0,  0,  0,  f_i);
}

Eigen::MatrixXf  RadarModel::getPhaseProbCond(double ph_i, double x_m, double y_m, double orientation_deg, double f_i){
    Eigen::MatrixXf PH_mat = getPhaseMat(x_m,y_m,orientation_deg, f_i);
    Eigen::MatrixXf ans = getProbCond(PH_mat, ph_i, _sigma_phase);
    
    return ans;
} 

Eigen::MatrixXf  RadarModel::getProbCond(Eigen::MatrixXf X_mat, double x, double sig){
      
  Eigen::MatrixXf likl_mat;
  
  // gaussian pdf
  // fddp(x,mu,sigma) = exp( -0.5 * ( (x - mu)/sigma )^2 )   /   (sigma * sqrt(2 pi )) 
  likl_mat = ( x - X_mat.array() ) /_sigma_power;
  likl_mat = -0.5 * ( likl_mat.array().pow(2.0) );
  likl_mat = likl_mat.array().exp() / ( _sigma_power * sqrt( 2.0 * M_PI ) ) ;

  return likl_mat;
}

Eigen::MatrixXf RadarModel::getPredictionStep(string tagLayerName, int step_size){
  int buffer_size = step_size;
  Eigen::MatrixXf X_mat = _rfid_belief_maps[tagLayerName];
  Eigen::MatrixXf result_mat(X_mat.rows(), X_mat.cols());
  Eigen::MatrixXf obst_mat = _rfid_belief_maps["ref_map"];
  double euclidean_distance, belief;
  grid_map::Index index;
  Position point;
  
  // Iterate on the map
  for (GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd();
       ++iterator) {
    const Index index(*iterator);
    _rfid_belief_maps.getPosition(*iterator, point);
    // If the cell is inside the map and not an obstacle
    if (_rfid_belief_maps.isInside(point) and _rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
      
      if (index(0) > buffer_size and index(0) <= _rfid_belief_maps.getLength().x() - buffer_size) {
        if (index(1) > buffer_size and index(1) <= _rfid_belief_maps.getLength().y() - buffer_size) {
          // Keep track of the new belief
          belief = X_mat(index(0), index(1));
          // Iterate on the squared filter (buffer_size * buffer_size)
          Index submapStartIndex(index(0) - buffer_size, index(1) - buffer_size);
          Index submapBufferSize(buffer_size, buffer_size);
          for (grid_map::SubmapIterator sub_iterator(
                  _rfid_belief_maps, submapStartIndex, submapBufferSize);
              !sub_iterator.isPastEnd(); ++sub_iterator) {
            Index sub_index(*sub_iterator);
            euclidean_distance = sqrt(pow(sub_index(0)-index(0), 2) + pow(sub_index(1)-index(1),2));
            belief += X_mat(sub_index(0), sub_index(1)) * exp(-0.5 * pow(euclidean_distance,2) / buffer_size);
          }
          result_mat(index(0), index(1)) = belief;
        }
      }
    }
  }

 

  // for (int row = buffer_size; row < X_mat.rows() - buffer_size; row++){
  //   for (int col = buffer_size; col < X_mat.cols() - buffer_size; col++){
  //     belief = X_mat(row, col);
  //     //Update only cells which are not obstacles
  //     index(0) = row;
  //     index(1) = col;
  //     _rfid_belief_maps.getPosition(index, point);
  //     if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val){
  //       // Iterate on the buffer size of the neighbors
  //       for (int i=row-buffer_size; i<row+buffer_size; i++){
  //         for (int j=col-buffer_size; j<col+buffer_size; j++){
  //           euclidean_distance = sqrt(pow(i-row, 2) + pow(j-col,2));
  //           belief += X_mat(i,j) * exp(-0.5 * pow(euclidean_distance,2) / buffer_size);
  //         }
  //       }
  //       result_mat(row, col) = belief;
  //     }
  //   }
  // }

return result_mat;
  // // Normalise the final matrix
  // result_mat = result_mat/result_mat.sum();
}

Eigen::MatrixXf RadarModel::getFriisMat(double x_m, double y_m, double orientation_deg, double freq){
  if (useFast)
    return getFriisMatFast(x_m, y_m, orientation_deg, freq);
  else
    return getFriisMatSlow(x_m, y_m, orientation_deg, freq);
}


Eigen::MatrixXf RadarModel::getFriisMatSlow(double x_m, double y_m, double orientation_deg, double freq){
  Eigen::MatrixXf rxPw_mat;
  double tag_x, tag_y, rxP;
  Position glob_point;
  Index ind;

  Size siz = _rfid_belief_maps.getSize();
  rxPw_mat = Eigen::MatrixXf(siz(0), siz(1));
  double orientation_rad = orientation_deg * M_PI / 180.0;

  // Obtain rel dist and friis to all points
  // MFC: can I turn this iteration into matrix operations?
  for (grid_map::GridMapIterator iterator(_rfid_belief_maps);
       !iterator.isPastEnd(); ++iterator) {
    // matrix indexes...
    ind = *iterator;
    // get cell center of the cell in the map frame.
    _rfid_belief_maps.getPosition(ind, glob_point);
    // that is where the tag supposedly is in map coordinates
    tag_x = glob_point.x();
    tag_y = glob_point.y();

    rxP = received_power_friis_with_obstacles(x_m, y_m, orientation_rad, tag_x,
                                              tag_y, 0, freq);
    rxPw_mat(ind(0), ind(1)) = rxP;
  }
  return rxPw_mat;
}

Eigen::MatrixXf RadarModel::getFriisMatFast(double x_m, double y_m, double orientation_deg, double freq){
  // https://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
  // https://github.com/ANYbotics/grid_map


  Eigen::MatrixXf X, Y, R, A, propL, antL,totalLoss, rxPower;
  Eigen::VectorXf x,y;
  Index i00,i0M,iNM,iN0, iRobot;
  double lambda =  C/freq;
  double orientation_rad = orientation_deg * M_PI/180.0;
  
  // rotate and translate
  Eigen::MatrixXf X0 = _rfid_belief_maps["X"];
  Eigen::MatrixXf Y0 = _rfid_belief_maps["Y"];
  
  double cA =cos(orientation_rad);
  double sA =sin(orientation_rad);

  X =   ( X0 * cA + Y0 * sA).array() - (x_m*cA + y_m*sA);
  Y =   (-X0 * sA + Y0 * cA).array() + (x_m*sA - y_m*cA);  

  // create R,Ang matrixes
  R = (X.array().square() + Y.array().square()).array().sqrt();
  A = Y.binaryExpr(X, std::ptr_fun(atan2f)).array();

  // 1. Create a friis losses propagation matrix without taking obstacles        
  auto funtor = std::bind(&SplineFunction::interpRadf, _antenna_gains, _1) ;
  antL =  TAG_LOSSES + A.unaryExpr( funtor ).array();   
  propL = LOSS_CONSTANT - (20.0 * (R * freq).array().log10()).array() ;

  // signal goes from antenna to tag and comes back again, so we double the losses
  totalLoss =  2.0*antL + 2.0*propL ;
  
  rxPower = totalLoss.array() + _txtPower;
  // this should remove points where friis is not applicable
  rxPower = (R.array()>2.0*lambda).select(rxPower,_txtPower); 
  
  //2. Create a NaN filled matrix for obstacles loses
  _rfid_belief_maps.add("obst_losses",NAN);

  // iterate over four lines to fill obst_losses layer ...................  
  // line 1: (0,0) to (0,M)
  i00 =grid_map::Index(0,0);
  i0M =grid_map::Index(0,_Ncol-1);
  _rfid_belief_maps.getIndex(Position( x_m, y_m), iRobot);
  addLossesTillEdgeLine(i00, i0M, iRobot );

  // line 2: (0,M) to (N,M)
  iNM =grid_map::Index(_Nrow-1,_Ncol-1);
  addLossesTillEdgeLine(i0M, iNM, iRobot );

  // line 3: (N,M) to (N,0)
  iN0 =grid_map::Index(_Nrow-1,0);
  addLossesTillEdgeLine(iNM, iN0, iRobot );

  // line 4: (N,0) to (0,0)
  addLossesTillEdgeLine(iN0, i00, iRobot );

  // And finally add obstacle losses and propagation losses  
  rxPower = rxPower   - _rfid_belief_maps.get("obst_losses");
  //std::cout << "Still running at line: " << __LINE__<< std::endl;

  // this should remove points where received power is too low
  rxPower = (rxPower.array()>SENSITIVITY).select(rxPower,SENSITIVITY); 

  // mfc trick used to see temp matrixes as images
  //_rfid_belief_maps.add("obst_losses",rxPower);
  return rxPower;

}

void RadarModel::addLossesTillEdgeLine(Index edge_index_start,   Index edge_index_end,   Index antenna_index){
  Index edge_index;
  
  double obst_loss_ray, obst_cell_inc;

  // Each "wall" adds around 3dB losses. A wall is ~15cm thick, then each cell adds  (3 * resolution / 0.15) dB losses
  obst_cell_inc = 20.0 * _resolution; // db 

  // move along the map edge defined by those two indexes
  for (grid_map::LineIterator edge_iterator(_rfid_belief_maps, edge_index_start, edge_index_end); !edge_iterator.isPastEnd(); ++edge_iterator) {
    edge_index = *edge_iterator; 
    //  - initializate cummulated losses to 0.
    obst_loss_ray = 0;

    //Now iterate from xm,ym to the point xi,yi in the edge
    for (grid_map::LineIterator loss_ray_iterator(_rfid_belief_maps, antenna_index, edge_index); !loss_ray_iterator.isPastEnd(); ++loss_ray_iterator) {

      // if the cell is obstacle, add L to cummulated_L
      if (( _rfid_belief_maps.at("ref_map", *loss_ray_iterator) != _free_space_val  )){
        obst_loss_ray += obst_cell_inc;
      }

      // obstacles losses in cell is cummulated_L. Avoid multiple edits   
      if (( _rfid_belief_maps.at("obst_losses", *loss_ray_iterator) != NAN  )){
        _rfid_belief_maps.at("obst_losses", *loss_ray_iterator) = obst_loss_ray; 
      }      

    }
  }

}

Eigen::MatrixXf RadarModel::getPhaseMat(double x_m, double y_m, double orientation_deg, double freq){
  
  Eigen::MatrixXf rxPh_mat;
  double tag_x, tag_y, tag_r, tag_h, rxPh;
  Position glob_point;
  Index ind;

  Size siz = _rfid_belief_maps.getSize();
  rxPh_mat = Eigen::MatrixXf(siz(0), siz(1));
  double orientation_rad = orientation_deg * M_PI / 180.0;

  // Obtain rel dist and to all points
  // MFC: can I turn this iteration into matrix operations?
  for (grid_map::GridMapIterator iterator(_rfid_belief_maps);
       !iterator.isPastEnd(); ++iterator) {
    // matrix indexes...
    ind = *iterator;
    // get cell center of the cell in the map frame.
    _rfid_belief_maps.getPosition(ind, glob_point);
    // that is where the tag supposedly is in map coordinates
    tag_x = glob_point.x();
    tag_y = glob_point.y();

    // now get robot - tag relative pose!
    double delta_x = (tag_x - x_m);
    double delta_y = (tag_y - y_m);
    // rotate
    tag_x = delta_x * cos(orientation_rad) + delta_y * sin(orientation_rad);
    tag_y = -delta_x * sin(orientation_rad) + delta_y * cos(orientation_rad);

    getSphericCoords(tag_x, tag_y, tag_r, tag_h);

    rxPh = phaseDifference(tag_r, tag_h, freq);

    rxPh_mat(ind(0), ind(1)) = rxPh;
  }
  return rxPh_mat;
}

void RadarModel::getImage(std::string layerName, std::string fileURI){
    //std::cout << "Still running at line: " << __LINE__<< std::endl;

    getImage(&_rfid_belief_maps, layerName, fileURI);
}

void RadarModel::getImage(GridMap* gm,std::string layerName, std::string fileURI){
  // Convert to image.
  cv::Mat image = layerToImage(gm, layerName);
  cv::imwrite( fileURI, image );
}

double RadarModel::getTotalWeight(int tag_i) {
  return getTotalWeight((_Ncol - 1) * (_resolution / 2.0),
                        (_Nrow - 1) * (_resolution / 2.0), 0,
                        _Ncol * _resolution, _Nrow * _resolution, tag_i);
}

double RadarModel::getTotalWeight(double x, double y, double orientation,
                                  double size_x, double size_y, int tag_i) {
  // TODO: I'm not using the orientation. Maybe it would be better to use a
  // polygon iterator,
  //     so we can rotate edges around the center and have a more flexible thing

  // submapStartIndex the start index of the submap, typically top-left index.
  grid_map::Index submapStartIndex, submapEndIndex, submapBufferSize;
  grid_map::Position submapStartPosition(x + (size_x / 2), y + (size_y / 2));
  grid_map::Position submapEndPosition(x - (size_x / 2), y - (size_y / 2));

  if (!_rfid_belief_maps.getIndex(submapStartPosition, submapStartIndex)) {
    submapStartIndex = grid_map::Index(0, 0);
    // std::cout<<"Clip start!" << std::endl;
  }

  if (!_rfid_belief_maps.getIndex(submapEndPosition, submapEndIndex)) {
    Size siz = _rfid_belief_maps.getSize();
    submapEndIndex = grid_map::Index(siz(0) - 1, siz(1) - 1);
    // std::cout<<"Clip end!" << std::endl;
  }

  submapBufferSize = submapEndIndex - submapStartIndex;

  grid_map::SubmapIterator iterator(_rfid_belief_maps, submapStartIndex,
                                    submapBufferSize);

  // std::cout<<"\nGet prob.:" << std::endl;
  // std::cout<<" Centered at Position (" << x << ", " << y << ") m. / Size ("
  // << size_x << ", " << size_y << ")" << std::endl; std::cout<<" Start pose ("
  // << submapStartPosition(0) << ", " << submapStartPosition(1) << ") m. to
  // pose " << submapEndPosition(0) << ", " << submapEndPosition(1) << ") m."<<
  // std::endl; std::cout<<" Start Cell ("  << submapStartIndex(0) << ", " <<
  // submapStartIndex(1) << ") to cell("  << submapEndIndex(0) << ", " <<
  // submapEndIndex(1) << ")"<< std::endl;

  return getTotalWeight(x, y, orientation, iterator, tag_i);
}

double RadarModel::getTotalWeight(double x, double y, double orientation,
                                  grid_map::SubmapIterator iterator,
                                  int tag_i) {

  double total_weight;
  Position point;

  std::string tagLayerName = getTagLayerName(tag_i);

  total_weight = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        total_weight += _rfid_belief_maps.atPosition(tagLayerName, point);
      }
    }
  }
  return total_weight;
}

void RadarModel::getImageDebug(GridMap *gm, std::string layerName,
                               std::string fileURI) {
  // plots circles in the edges of the corresponding layer

  // Convert to image.
  cv::Mat image = layerToImage(gm, layerName);
  
  cv::Scalar green( 0, 255, 0 );
  cv::Scalar blue( 255, 0, 0 );      
  cv::Scalar red( 0, 0, 255 );
  cv::Scalar yellow( 0, 255, 255 );
  
  grid_map::Index index;

  double maxX = (*gm).getLength().x() / 2;
  double maxY = (*gm).getLength().y() / 2;

  grid_map::Position p(maxX, maxY);
  (*gm).getIndex(p, index);
  cv::Point gree(index.x(), index.x());
  std::cout << "Green: (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" << std::endl;

  p = Position(maxX, -maxY);
  (*gm).getIndex(p, index);
  cv::Point blu(index.y(), index.x());
  std::cout << "Blue (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" << std::endl;

  p = Position(-maxX, -maxY);
  (*gm).getIndex(p, index);
  cv::Point re(index.y(), index.x());
  std::cout << "Red (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" << std::endl;

  p = Position(-maxX, maxY);
  (*gm).getIndex(p, index);
  cv::Point yell(index.y(), index.x());
  std::cout << "Yellow (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" << std::endl;

  cv::circle(image, gree, 20, green, -1);
  cv::circle(image, blu, 20, blue, -1);
  cv::circle(image, re, 20, red, -1);
  cv::circle(image, yell, 20, yellow, -1);

  cv::Point triang_points[1][3];
  double h = 0.2;

  p = Position(h / 2.0, 0);
  (*gm).getIndex(p, index);
  triang_points[0][0] = cv::Point(index.y(), index.x());
  std::cout << "p (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" << std::endl;

  p = Position(-h / 2.0, -h / 2.0);
  (*gm).getIndex(p, index);
  triang_points[0][1] = cv::Point(index.y(), index.x());
  std::cout << "p (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" << std::endl;

  p = Position(-h / 2.0, h / 2.0);
  (*gm).getIndex(p, index);
  triang_points[0][2] = cv::Point(index.y(), index.x());
  std::cout << "p (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" << std::endl;

  const cv::Point *ppt[1] = {triang_points[0]};
  int npt[] = {3};
  cv::fillPoly(image, ppt, npt, 1, red, 8);

  // Rotate 90 Degrees Clockwise To get our images to have increasing X to right
  // and increasing Y up
  cv::transpose(image, image);
  cv::flip(image, image, 1);
  cv::imwrite(fileURI, image);
}

std::string RadarModel::getPowLayerName(double freq_i) {
  return "P_" + getLayerName(freq_i / 1e6);
}

std::string RadarModel::getPhaseLayerName(double freq_i) {
  return "D_" + getLayerName(freq_i / 1e6);
}

std::string RadarModel::getTagLayerName(int tag_num) {
  return std::to_string(tag_num);
}

std::string RadarModel::getLayerName(double x) {

  // Create an output string stream
  std::ostringstream streamObj3;

  // Set Fixed -Point Notation
  streamObj3 << std::fixed;

  // Set precision to 2 digits
  streamObj3 << std::setprecision(2);

  // Add double to stream
  streamObj3 << x;

  // Get string from output string stream
  std::string strObj3 = streamObj3.str();

  return strObj3;
}

cv::Point RadarModel::getPoint(double x_m, double y_m) {
  Length mlen = _rfid_belief_maps.getLength();
  grid_map::Position orig = _rfid_belief_maps.getPosition();
  double min_x = orig(0) - mlen(0) / 2 + _resolution;
  double max_x = orig(0) + mlen(0) / 2 - _resolution;
  double min_y = orig(1) - mlen(1) / 2 + _resolution;
  double max_y = orig(1) + mlen(1) / 2 - _resolution;

  grid_map::Index index;

  grid_map::Position p(x_m, y_m);

  if (!_rfid_belief_maps.getIndex(p, index)) {
    // std::cout<<" Position ("  << p(0) << ", " << p(1) << ") is out of
    // _rfid_belief_maps bounds!!" <<std::endl;
    x_m = std::min(std::max(x_m, min_x), max_x);
    y_m = std::min(std::max(y_m, min_y), max_y);
    p = grid_map::Position(x_m, y_m);
    _rfid_belief_maps.getIndex(p, index);
  }

  // cast from gridmap indexes to opencv indexes
  int cv_y = (_Nrow - 1) - index.x();
  int cv_x = (_Ncol - 1) - index.y();
  // std::cout<<"Which equals to opencv cell ("  << cv_x << ", " << cv_y << ") "
  // << std::endl;

  return cv::Point(cv_x, cv_y);
}

void RadarModel::getSphericCoords(double x, double y, double &r, double &phi) {
  r = sqrt(x * x + y * y);
  phi = atan2(y, x);
}

//////////////////////////  Print and visualization
///////////////////////////////////////////////////////


void RadarModel::PrintRecPower(std::string fileURI, double f_i){
  PrintRecPower(fileURI, 0,0,0,  f_i);
}

void RadarModel::PrintRecPower(std::string fileURI,double x_m, double y_m, double orientation_deg,  double f_i){
  // create a copy of the average values
  //std::cout << "Still running at line: " << __LINE__<< std::endl;
  Eigen::MatrixXf av_mat = getFriisMat(x_m, y_m, orientation_deg, f_i);

  //std::cout << "Still running at line: " << __LINE__<< std::endl;
  PrintProb(fileURI, &av_mat);
  //std::cout << "Still running at line: " << __LINE__<< std::endl;
}

void RadarModel::PrintPhase(std::string fileURI,  double f_i){            
  PrintPhase(fileURI, 0,0,0, f_i);    
}

void RadarModel::PrintPhase(std::string fileURI,double x_m, double y_m, double orientation_deg,  double f_i){        
  // create a copy of the average values
  Eigen::MatrixXf av_mat = getPhaseMat(x_m, y_m, orientation_deg, f_i);
  PrintProb(fileURI, &av_mat);
}

void RadarModel::PrintPowProb(std::string fileURI, double rxPw, double x_m, double y_m, double orientation_deg, double f_i){
  Eigen::MatrixXf prob_mat = getPowProbCond(rxPw, x_m, y_m, orientation_deg, f_i);
  PrintProb(fileURI, &prob_mat);
}

void RadarModel::PrintPowProb(std::string fileURI, double rxPw, double f_i){
  PrintPowProb(fileURI, rxPw, 0,0,0, f_i);
}

void RadarModel::PrintPhaseProb(std::string fileURI, double phi, double f_i){
  PrintPhaseProb(fileURI, phi, 0, 0, 0, f_i);
}

void RadarModel::PrintPhaseProb(std::string fileURI, double phi, double x_m, double y_m, double orientation_deg, double f_i){
  Eigen::MatrixXf prob_mat = getPhaseProbCond(phi, x_m, y_m, orientation_deg, f_i);
  PrintProb(fileURI, &prob_mat);
}

void RadarModel::PrintBothProb(std::string fileURI, double rxPw, double phi, double f_i){
  PrintBothProb(fileURI, rxPw, phi, 0,0,0, f_i);
}

void RadarModel::PrintBothProb(std::string fileURI, double rxPw, double phi, double x_m, double y_m, double orientation_deg, double f_i){
  Eigen::MatrixXf prob_mat = getPowProbCond(rxPw, x_m, y_m, orientation_deg, f_i).cwiseProduct(getPhaseProbCond(phi, x_m, y_m, orientation_deg, f_i));
  prob_mat = prob_mat/prob_mat.sum();
  PrintProb(fileURI, &prob_mat);
}

void RadarModel::PrintProb(std::string fileURI, Eigen::MatrixXf *prob_mat) {
  PrintProb(fileURI, prob_mat, prob_mat->rows(), prob_mat->cols(), _resolution);
}

void RadarModel::PrintProb(std::string fileURI, Eigen::MatrixXf* prob_mat, double sX, double sY, double res){
  GridMap tempMap;      
  tempMap.setGeometry(Length(sX, sY), res);
  tempMap.add("res", *prob_mat);
  getImage(&tempMap, "res", fileURI);
}

void RadarModel::PrintRefMapWithTags(std::string fileURI) {
  std::vector<std::pair<cv::Scalar, std::string>> color_list;

  // Some color  ...
  cv::Scalar red(0, 0, 255);
  color_list.push_back(std::make_pair(red, "red"));
  cv::Scalar green(0, 255, 0);
  color_list.push_back(std::make_pair(green, "green"));
  cv::Scalar blue(255, 0, 0);
  color_list.push_back(std::make_pair(blue, "blue"));
  cv::Scalar yellow(0, 255, 255);
  color_list.push_back(std::make_pair(yellow, "yellow"));
  cv::Scalar blueviolet(226, 138, 43);
  color_list.push_back(std::make_pair(blueviolet, "blueviolet"));
  cv::Scalar turquoise(224, 208, 64);
  color_list.push_back(std::make_pair(turquoise, "turquoise"));
  cv::Scalar orange(165, 0, 255);
  color_list.push_back(std::make_pair(orange, "orange"));
  cv::Scalar pink(192, 203, 255);
  color_list.push_back(std::make_pair(pink, "pink"));
  cv::Scalar chocolate(105, 30, 210);
  color_list.push_back(std::make_pair(chocolate, "chocolate"));
  cv::Scalar dodgerblue(144, 255, 30);
  color_list.push_back(std::make_pair(dodgerblue, "dodgerblue"));

  // Convert to image.
  cv::Mat image = rfidBeliefToCVImg("ref_map");

  cv::Point center;

  for (int i = 0; i < _tags_coords.size(); i++) {
    double x = _tags_coords[i].first;
    double y = _tags_coords[i].second;
    center = getPoint(x, y);
    cv::circle(image, center, 5, color_list[i % color_list.size()].first, -1);
  }
  /// overlay active map edges
  /// .............................................................................................
  overlayMapEdges(image);

  cv::imwrite(fileURI, image);
}

void RadarModel::overlayMapEdges(cv::Mat image) {
  cv::Point square_points[1][4];
  cv::Scalar red(0, 255, 255);
  int cv_y, cv_x;

  cv_y = (_Nrow - 1);
  cv_x = (_Ncol - 1);

  square_points[0][0] = cv::Point(0, 0);
  square_points[0][1] = cv::Point(0, cv_y);
  square_points[0][2] = cv::Point(cv_x, cv_y);
  square_points[0][3] = cv::Point(cv_x, 0);

  const cv::Point *pts[1] = {square_points[0]};
  int npts[] = {4};
  cv::polylines(image, pts, npts, 1, true, red);
}

void RadarModel::overlayRobotPoseT(double robot_x, double robot_y,
                                   double robot_head, cv::Mat &image) {
  grid_map::Index index;
  cv::Point center;
  grid_map::Position p;
  cv::Point pentag_points[1][5];

  cv::Scalar red(0, 0, 255);
  center = getPoint(robot_x, robot_y);

  // create a pentagone pointing x+

  int h = 4; // pixels?

  pentag_points[0][0] = cv::Point(center.x - h, center.y - h);
  pentag_points[0][1] = cv::Point(center.x - h, center.y + h);
  pentag_points[0][2] = cv::Point(center.x, center.y + 2 * h);
  pentag_points[0][3] = cv::Point(center.x + h, center.y + h);
  pentag_points[0][4] = cv::Point(center.x + h, center.y - h);
  const cv::Point *pts[1] = {pentag_points[0]};
  rotatePoints(pentag_points[0], 5, center.x, center.y, robot_head);
  int npts[] = {5};
  cv::fillPoly(image, pts, npts, 1, red, 8);
}

void RadarModel::overlayRobotPose(double robot_x, double robot_y,
                                  double robot_head, cv::Mat &image) {
  cv::Point center;

  cv::Scalar red(0, 0, 255);
  center = getPoint(robot_x, robot_y);
  cv::circle(image, center, 5, red, -1);
}

void RadarModel::rotatePoints(cv::Point *points, int npts, int cxi, int cyi,
                              double ang) {
  double offsetx, offsety, cx, cy, px, py, cosA, sinA;

  cx = (double)cxi;
  cy = (double)cyi;

  cosA = cos(ang);
  sinA = sin(ang);

  for (int i = 0; i < npts; i++) {
    px = points[i].x;
    py = points[i].y;

    offsetx = (cosA * (px - cx)) + (sinA * (py - cy));
    offsety = -(sinA * (px - cx)) + (cosA * (py - cy));

    points[i].x = ((int)offsetx) + cxi;
    points[i].y = ((int)offsety) + cyi;
  }
}

cv::Mat RadarModel::rfidBeliefToCVImg(std::string layer_i){
  return layerToImage(&_rfid_belief_maps,layer_i);
}

cv::Mat  RadarModel::layerToImage(GridMap* gm,std::string layerName){
  cv::Mat image;
  const float minValue = (*gm)[layerName].minCoeff();
  const float maxValue = (*gm)[layerName].maxCoeff();
  GridMapCvConverter::toImage<unsigned char, 3>(*gm, layerName, CV_8UC3, minValue, maxValue, image);  
  cv::flip(image, image, -1);
  return image;
}

grid_map::Position RadarModel::fromPoint(cv::Point cvp) {
  grid_map::Position p;
  int gm_x, gm_y;

  // cast from opencv indexes to  gridmap indexes
  gm_x = (_Nrow - 1) - cvp.y;
  gm_y = (_Ncol - 1) - cvp.x;

  grid_map::Index index(gm_x, gm_y);

  if (!_rfid_belief_maps.getPosition(index, p)) {
    // std::cout<<" Index ("  << index(0) << ", " << index(1) << ") is out of
    // _rfid_belief_maps bounds!!" <<std::endl;
    gm_x = std::min(std::max(gm_x, 0), _Nrow - 1);
    gm_y = std::min(std::max(gm_y, 0), _Ncol - 1);
    index = grid_map::Index(gm_x, gm_y);
    _rfid_belief_maps.getPosition(index, p);
  }

  return p;
}

//////////////////////////// Other functions ////////////////////////////

void RadarModel::debugInfo() {
  std::cout << ".................." << std::endl;
  Length mlen = _rfid_belief_maps.getLength();
  std::cout << "RFID belief area is: " << mlen(0) << " by " << mlen(1)
            << " m. (x,y)" << std::endl;
  std::cout << "Resolution is: " << _resolution << " m. /cell" << std::endl;
  std::cout << "RFID belief grid is: " << _rfid_belief_maps.getSize()(0)
            << " by " << _rfid_belief_maps.getSize()(1) << " cells (i,j axis)"
            << std::endl;
  std::cout << ". " << std::endl;
  std::cout << "Total grid Map is: " << _Ncol << " by " << _Nrow
            << " cells (i,j axis)" << std::endl;
  std::cout << ".................." << std::endl;
}

void RadarModel::saveProbMapDebug(std::string savePATH, int tag_num, int step,
                                  double robot_x, double robot_y,
                                  double robot_head) {
  char buffer[10];
  int n;

  std::string fileURI = savePATH + "T";

  n = sprintf(buffer, "%01d", tag_num);
  fileURI += std::string(buffer) + "_S";
  n = sprintf(buffer, "%03d", step);
  fileURI += std::string(buffer) + "_tempMap.png";

  std::string layerName = getTagLayerName(tag_num);
  // Some color  ...
  cv::Scalar green(0, 255, 0);

  // Convert to image.
  cv::Mat image = rfidBeliefToCVImg(layerName);

  cv::Point tag_center;

  /// overlay tag position
  /// .................................................................................................
  double tx = _tags_coords[tag_num].first;
  double ty = _tags_coords[tag_num].second;
  tag_center = getPoint(tx, ty);
  cv::circle(image, tag_center, 5, green, 1);

  /// overlay robot position
  /// .................................................................................................
  overlayRobotPoseT(robot_x, robot_y, robot_head, image);

  // and save
  cv::imwrite( fileURI, image );

  // mfc trick used to see temp matrixes as images
  // add on ......................................................................................................
  // if (_rfid_belief_maps.exists("obst_losses")){
  //   image = rfidBeliefToCVImg("obst_losses");
  //   // overlay tag position 
  //   cv::circle(image, tag_center , 5, green, 1);
  //   // overlay robot position 
  //   overlayRobotPoseT(robot_x, robot_y, robot_head, image);
    
  //   fileURI = savePATH + "T";
  //   n=sprintf (buffer, "%01d", tag_num);
  //   fileURI += std::string(buffer) +"_rxPower_S";
  //   n=sprintf (buffer, "%03d", step);
  //   fileURI += std::string(buffer)+ ".png";

  //   cv::imwrite( fileURI, image );
  // }
}

void RadarModel::clearObstacles(cv::Mat &image) {
  // MFC this method does not work .... dont use it!
  int b = 0;
  int a = 1 / b;
  //////////////////////////////////

  cv::Vec<unsigned char, 3> red = {0, 0, 255};

  // Initialize result image.
  cv::Mat result = image.clone().setTo(cv::Scalar(255, 255, 255));

  // Convert to image.
  cv::Mat ref_img = rfidBeliefToCVImg("ref_map");

  uint8_t ref_val;
  // Copy pixels from background to result image, where pixel in mask is 0.
  for (int x = 0; x < image.size().width; x++) {
    for (int y = 0; y < image.size().height; y++) {
      ref_val = ref_img.at<uint8_t>(y, x);
      if (ref_val == 0) {
        result.at<cv::Vec3b>(y, x) = red;
      } else {
        result.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(y, x);
      }
      if ((ref_val != 0) && (ref_val != 255)) {
        std::cout << "Map image NOT BINARY AT (" << x << ", " << y
                  << ") = " << ref_val << std::endl;
      }
    }
  }

  // there you go ...
  image = result;

  // std::cout<<"Map image (" << ref_img.size().width << ", "
  // <<ref_img.size().height<<") pixels" <<std::endl; std::cout<<"Out image ("
  // << image.size().width << ", " <<image.size().height<<") pixels"
  // <<std::endl; and save
  cv::imwrite("/tmp/refMap.png", ref_img);

  // cv::Mat rgba;

  // // First create the image with alpha channel
  // cv::cvtColor(image, rgba , cv::cv::COLOR_GRAY2RGBA);

  // // Split the image for access to alpha channel
  // std::vector<cv::Mat>channels(4);
  // cv::split(rgba, channels);

  // // Assign the mask to the last channel of the image
  // channels[3] = alpha_data;

  // // Finally concat channels for rgba image
  // cv::merge(channels, 4, rgba);
}

std::vector<double> RadarModel::range(double start, double stop, double step) {
  std::vector<double> ans;
  double currVal = start;
  while (currVal <= stop) {
    ans.push_back(currVal);
    currVal += step;
  }
  return ans;
}

double RadarModel::antennaPlaneLoss(double angleRad) {
  double g_ans;
  double g_i = -22.6;
  double g_p = -22.6;
  int i;
  int p;
  double ang_i = 0;
  double ang_p = 0;
  double m = 0;
  double g_o = 0;
  double angleDeg = angleRad * 180.0 / M_PI;

  if (fabs(angleDeg) > 180.0) {
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "Angle (" << angleDeg << ") deg. " << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  }

  // gain list entries start at -180 degrees to 180 in steps of 15.
  double fIndex = (angleDeg + 165.0) / 15.0;

  // we dont want to index out of the range ...
  i = std::max(std::min((int)ceil(fIndex), 24), 0);
  p = std::max(std::min((int)floor(fIndex), 24), 0);

  ang_i = (i * 15.0) - 180.0;
  ang_p = (p * 15.0) - 180.0;

  g_i = ANTENNA_LOSSES_LIST[i];
  g_p = ANTENNA_LOSSES_LIST[p];

  if (i != p) {
    m = (g_i - g_p) / (ang_i - ang_p);
  }

  g_o = g_i - (m * ang_i);
  g_ans = m * angleDeg + g_o;

  //

  return g_ans;
}

float RadarModel::sign(float x) {
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return 0.0;
}

double RadarModel::received_power_friis_with_obstacles(
    double antenna_x, double antenna_y, double antenna_h, double tag_x,
    double tag_y, double tag_h, double freq) {
  return received_power_friis_with_obstacles(antenna_x, antenna_y, antenna_h,
                                             tag_x, tag_y, tag_h, freq,
                                             _txtPower, _antenna_gains);
}

double RadarModel::received_power_friis_with_obstacles(
    double antenna_x, double antenna_y, double antenna_h, double tag_x,
    double tag_y, double tag_h, double freq, double txtPower,
    SplineFunction antennaGainsModel) {

  double rel_tag_x, rel_tag_y, rel_tag_r, rel_tag_h, rxP, wall_losses, delta_x,
      delta_y;
  int count_obs_cell;

  // Get robot - tag relative pose
  delta_x = (tag_x - antenna_x);
  delta_y = (tag_y - antenna_y);

  // rotate
  rel_tag_x = delta_x * cos(antenna_h) + delta_y * sin(antenna_h);
  rel_tag_y = -delta_x * sin(antenna_h) + delta_y * cos(antenna_h);

  getSphericCoords(rel_tag_x, rel_tag_y, rel_tag_r, rel_tag_h);

  rxP = received_power_friis_polar(rel_tag_r, rel_tag_h, freq, txtPower,
                                   antennaGainsModel);

  if (rxP > SENSITIVITY) {
    // Check if there is line of sight between antenna and tag
    Index antenna_index;
    Index tag_index;
    _rfid_belief_maps.getIndex(Position(antenna_x, antenna_y), antenna_index);
    _rfid_belief_maps.getIndex(Position(tag_x, tag_y), tag_index);

    count_obs_cell = 0;
    wall_losses = 0;

    for (grid_map::LineIterator iterator(_rfid_belief_maps, antenna_index,
                                         tag_index);
         !iterator.isPastEnd(); ++iterator) {
      if ((_rfid_belief_maps.at("ref_map", *iterator) != _free_space_val)) {
        count_obs_cell++;
      }
    }

    // Each "wall" adds around 3dB losses. A wall is ~15cm thick, then each cell
    // adds  3 * resolution / 0.15 dB losses
    wall_losses = 20.0 * _resolution * count_obs_cell;

    rxP = rxP - wall_losses;
  }

  if (rxP < SENSITIVITY) {
    rxP = SENSITIVITY;
  }

  return rxP;
}

double
RadarModel::received_power_friis_polar(double tag_r, double tag_h, double freq,
                                       double txtPower,
                                       SplineFunction antennaGainsModel) {
  double rxPower = txtPower;
  double ant1, antL, propL;
  double lambda = C / freq;
  // otherwise friis approach does not apply
  if (tag_r > 2 * lambda) {
    /*
    SIMPLIFICATION!!! TAG is OMNIDIRECTIONAL
    (a.k.a. don't have tag radiation pattern and
    Here they say it's ok
    https://www.hindawi.com/journals/ijap/2013/194145/tab4/
    */
    ant1 = antennaGainsModel.interpRad(tag_h);

    antL = TAG_LOSSES + ant1;

    // propagation losses
    propL = LOSS_CONSTANT - (20 * log10(tag_r * freq));
    // signal goes from antenna to tag and comes back again, so we double the
    // losses
    rxPower += 2 * antL + 2 * propL;
  }

  if (rxPower > txtPower) {
    std::cout << endl
              << endl
              << "ERROR! MORE POWER RECEIVED THAN TRANSMITTED!!!!!!!!!!!!! "
              << endl;
    std::cout << "Relative tag polar pose: (" << tag_r << " m.,"
              << tag_h * 180 / M_PI << " deg.)" << endl;
    std::cout << "At Freq.: (" << freq / 1e6 << " MHz.)" << endl;
    std::cout << "Lambda: (" << lambda << " m.)" << endl;
    std::cout << "Tx Pw.: (" << txtPower << " dB)" << endl;
    std::cout << "Rx Pw.: (" << rxPower << " dB)" << endl;
    std::cout << "Antenna losses: (" << antL << " dB)" << endl;
    std::cout << "Propagation losses: (" << propL << " dB)" << endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl
              << endl
              << endl;
  }

  return rxPower;
}

double RadarModel::phaseDifference(double tag_x, double tag_y, double freq) {
  double phi;
  double r;

  getSphericCoords(tag_x, tag_y, r, phi);
  double phase = PHASE_CONSTANT * freq * r;
  phase = fmod(phase, M_PI);
  return phase;
}

std::pair<int, std::pair<int, int>> RadarModel::findTagFromBeliefMap(int num_tag) {

  // Access the belief map of every tag
  std::string layerName = getTagLayerName(num_tag);
  GridMap::Matrix &grid = _rfid_belief_maps[layerName];

  std::pair<int, int> tag(0, 0);
  double powerRead = 0;
  int buffer_size = 3;
  
  for (GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd();
       ++iterator) {
    const Index index(*iterator);
    // For every cell, analyse the surrounding area
    double tmp_power = 0.0;
    tmp_power = grid(index(0), index(1));
    
    if (index(0) > buffer_size and
        index(0) <= _rfid_belief_maps.getLength().x() - buffer_size) {
      if (index(1) > buffer_size and
          index(1) <= _rfid_belief_maps.getLength().y() - buffer_size) {
        Index submapStartIndex(index(0) - buffer_size, index(1) - buffer_size);
        Index submapBufferSize(buffer_size, buffer_size);
        for (grid_map::SubmapIterator sub_iterator(
                 _rfid_belief_maps, submapStartIndex, submapBufferSize);
             !sub_iterator.isPastEnd(); ++sub_iterator) {
          Index sub_index(*sub_iterator);
          tmp_power += grid(sub_index(0), sub_index(1));
        }
      }
    }

    if (tmp_power > powerRead) {
      powerRead = tmp_power;
      tag.first = index(0);
      tag.second = index(1);
    }
  }
  
  // Normalise the tag coordinate to follow Ricc's system
  tag.first = (grid.rows() - tag.first)*_resolution;
  tag.second = (grid.cols() - tag.second)*_resolution;
  std::pair<int, std::pair<int, int>> final_return(powerRead, tag);

  return final_return;
}

void RadarModel::normalizeRFIDLayer(std::string layerName) {
  double totalW = _rfid_belief_maps[layerName].sum();
  if (totalW > 0) {
    _rfid_belief_maps[layerName] = _rfid_belief_maps[layerName] / totalW;
  }

  // double minW = _rfid_belief_maps[layerName].minCoeff();
  // double maxW = _rfid_belief_maps[layerName].maxCoeff();
  // _rfid_belief_maps.add("min", minW);
  // _rfid_belief_maps.add("max", minW);
  // _rfid_belief_maps[layerName] = (_rfid_belief_maps[layerName] -
  // _rfid_belief_maps["min"])/ (maxW - minW);
}

void RadarModel::normalizeRFIDMap() {
  std::string layerName;
  // For every tag map, calculate the sum over the pixels and
  // divide each pixel intensity by the sum
  for (int i = 0; i < _numTags; ++i) {
    layerName = getTagLayerName(i);
    double totalW = _rfid_belief_maps[layerName].sum();
    if (totalW > 0) {
      _rfid_belief_maps[layerName] = _rfid_belief_maps[layerName] / totalW;
    }
  }
}

void RadarModel::clearObstacleCellsRFIDMap() {
  Position point;
  std::string layerName;

  for (int i = 0; i < _numTags; ++i) {
    layerName = getTagLayerName(i);
    for (grid_map::GridMapIterator iterator(_rfid_belief_maps);
         !iterator.isPastEnd(); ++iterator) {
      // get cell center of the cell in the map frame.
      _rfid_belief_maps.getPosition(*iterator, point);
      if (_rfid_belief_maps.atPosition("ref_map", point) != _free_space_val) {
        _rfid_belief_maps.at(layerName, *iterator) = 0.0;
        // count++;
      }
    }
  }
}

///////////////  ADD MEASUREMENT METHOD ////////////////////////////////////////

// So, robot at pr (x,y,orientation) (long, long, int) receives
// rxPower,phase,freq from tag i .
// TODO: this is a simplistic model that just "ignores" walls: but they do have
// absortion and reduce received power at their locations...
void RadarModel::addMeasurement(double x_m, double y_m, double orientation_deg,
                                double rxPower, double phase, double freq,
                                string tagLayerName) {

  Eigen::MatrixXf rxPw_mat, likl_mat, gaussian_kernel, prediction_belief;
  // std::string tagLayerName;

  // tagLayerName = getTagLayerName(i);
  Size siz = _rfid_belief_maps.getSize();
  rxPw_mat = Eigen::MatrixXf(siz(0), siz(1));

  // CORRECTION STEP
  // get the expected power at each point
  rxPw_mat = getFriisMat(x_m, y_m, orientation_deg, freq);

  // get the likelihood of the received power at each point
  likl_mat = getProbCond(rxPw_mat, rxPower, _sigma_power);

  // Where X_mat is < than SENSITIVITY, the tag wont be... prob 0
  // likl_mat = (likl_mat.array()<=SENSITIVITY).select(0,likl_mat);

  // this should remove prob at obstacles
  Eigen::MatrixXf obst_mat = _rfid_belief_maps["ref_map"];
  likl_mat = (obst_mat.array() == _free_space_val).select(likl_mat, 0);

  // normalize in this space:
  double bayes_den = likl_mat.sum();
  if (bayes_den > 0) {
    likl_mat = likl_mat / bayes_den;
    // now do bayes ...  everywhere
    _rfid_belief_maps[tagLayerName] =
        _rfid_belief_maps[tagLayerName].cwiseProduct(likl_mat);
  }

  normalizeRFIDLayer(tagLayerName);
  // cout << "X_mat max coeff: " << _rfid_belief_maps[tagLayerName].maxCoeff() << endl;


  // PREDICTION STEP
  prediction_belief = getPredictionStep(tagLayerName, 2);
  // Get rid of obstacles
  prediction_belief = (obst_mat.array() == _free_space_val).select(prediction_belief, 0);
  // _rfid_belief_maps[tagLayerName] = _rfid_belief_maps[tagLayerName].cwiseProduct(prediction_belief);
  // Normalise 
  // prediction_belief = prediction_belief/prediction_belief.sum();
  _rfid_belief_maps[tagLayerName] = prediction_belief;
  // cout << "Max: " << prediction_belief.maxCoeff() << endl;
  normalizeRFIDLayer(tagLayerName);



  if(_probabilisticTag){
    // if rxPower < SENSITIVITY (no power received), we want to
    // add uncertainty on the tag position using Gaussian Random Walk~N(0, T*sigma)
    // where T is the number of timesteps we didn't received answers
    std::pair<int, std::pair<int, int>> power_tag;
    std::pair<int, int> tmp_belief_tag;
    
    // if (i == 0 ){  
      // cout << "R: " << rxPower << ", S:" << SENSITIVITY << endl;
      int i = std::stoi( tagLayerName );
      power_tag = findTagFromBeliefMap(i);
      tmp_belief_tag = power_tag.second;
    // }

    // We received power and it's the first time
    if(rxPower > SENSITIVITY && _first_detection[i] == false)
    {
      _first_detection[i] = true;
      
    }

    if(rxPower > SENSITIVITY){
      _iteration_no_readings[i] = 1;
      if (tmp_belief_tag.first != 0 and tmp_belief_tag.second != 0){
        // if (i == 0 ){  
          _belief_tags[i] = tmp_belief_tag;
          // cout <<"        --->[YES Reading]B:" << tmp_belief_tag.first << "," << tmp_belief_tag.second << endl;
        // }
      }
      
    }

    // We apply a gaussian kernel only if we don't receive nothing now
    // (rxPower < SENSITIVY) but already perceived 
    // the tag  (_first_detection == true)
    if (rxPower <= SENSITIVITY and _first_detection[i] == true){
      if ( _belief_tags[i].first != 0 and _belief_tags[i].second != 0 ){
        // std::pair<int, std::pair<int, int>> power_tag;
        // std::pair<int, int> _belief_tag;
        // power_tag = findTagFromBeliefMap(i);
        // cout <<"  --->[No Reading]B:" << _belief_tags[i].first << "," << _belief_tags[i].second << endl;
        // _belief_tag = power_tag.second;
        gaussian_kernel = getGaussianRandomWalk(_belief_tags[i], i);
        
        // this should remove prob at obstacles
        obst_mat = _rfid_belief_maps["ref_map"];
        likl_mat = (obst_mat.array() == _free_space_val).select(likl_mat, 0);
        // cout << "G: " << gaussian_kernel.cols() << endl;
        // cout << "L: " << likl_mat.cols() << endl;
        gaussian_kernel = gaussian_kernel.cwiseProduct(likl_mat);
        
        // cv::Mat kernel_mat = cv::Mat::zeros(_Nrow, _Ncol, CV_32F);
        // bool check;
        // cv::eigen2cv(gaussian_kernel, kernel_mat);
        // kernel_mat = 255*kernel_mat/gaussian_kernel.maxCoeff();
        // check = cv::imwrite("/home/pulver/Desktop/gk/gaussian_kernel_" + to_string(_counter) + ".jpg", kernel_mat);
        // cv::eigen2cv(_rfid_belief_maps["ref_map"], kernel_mat);
        // check = cv::imwrite("/home/pulver/Desktop/gk/map_.jpg", kernel_mat);

        // cout << "B: " << _rfid_belief_maps[tagLayerName].maxCoeff() << ", K: " << gaussian_kernel.maxCoeff() << endl;
        // 1) SUM
        _rfid_belief_maps[tagLayerName] += gaussian_kernel;

  //       // 2) Product
  //       // _rfid_belief_maps[tagLayerName] =
  //       //   _rfid_belief_maps[tagLayerName].cwiseProduct(gaussian_kernel);

  //       // 3) Second bayes using the new gaussian kernel as observation
  //       // normalizeRFIDLayer(tagLayerName);
  //       // bayes_den = gaussian_kernel.sum();
  //       // if (bayes_den > 0) {
  //       //   gaussian_kernel = gaussian_kernel / bayes_den;
  //       //   // now do bayes ...  everywhere
  //       //   _rfid_belief_maps[tagLayerName] =
  //       //       _rfid_belief_maps[tagLayerName].cwiseProduct(gaussian_kernel);
  //       // }

  //       // 4) Increase the white area adding a small probability in every cell
  //       // and normalising again
  //       // Eigen::MatrixXf tmp = Eigen::MatrixXf::Constant(siz(0), siz(1), 1.1);
  //       // _rfid_belief_maps[tagLayerName] = _rfid_belief_maps[tagLayerName].cwiseProduct(tmp);


          
        // cv::eigen2cv(_rfid_belief_maps[tagLayerName], kernel_mat);
        // kernel_mat = 255*kernel_mat/_rfid_belief_maps[tagLayerName].maxCoeff();
        // check = cv::imwrite("/home/pulver/Desktop/gk/conv_" + to_string(_counter) + ".jpg", kernel_mat);
        
        _iteration_no_readings[i]++;
        // _counter++;
        
        
      }
      
    }

    
    normalizeRFIDLayer(tagLayerName);
  }


}

double RadarModel::getMapTotalEntropy() {
  double totalEntropy = 0;
  double tmp = 0;
  Eigen::MatrixXf likelihood, negLikelihood, logLikelihood, logNegLikelihood,
      ones;
  string tagLayerName;
  Position point;
  grid_map::Index index;
  // For every layer, compute the entropy and then sum it up
  // std::chrono::steady_clock::time_point begin =
  // std::chrono::steady_clock::now(); #pragma omp parallel for
  // num_threads(omp_get_max_threads()) private(tagLayerName, likelihood,
  // negLikelihood, logLikelihood, logNegLikelihood) schedule(auto)
  // reduction(+:totalEntropy)
  for (int tagID = 0; tagID < _tags_coords.size(); tagID++) {
    tagLayerName = std::to_string(tagID);
    // h = -p * log(p) - (1-p) * log(1-p)
    likelihood = _rfid_belief_maps[tagLayerName];
    negLikelihood = 1 - likelihood.array();
    // ones = Eigen::MatrixXf::Ones(likelihood.rows(), likelihood.cols());
    logLikelihood = likelihood.array().log();
    logNegLikelihood = negLikelihood.array().log();

    logLikelihood = logLikelihood.array().isInf().select(0.0, logLikelihood);
    logNegLikelihood =
        logNegLikelihood.array().isInf().select(0.0, logNegLikelihood);

    // logLikelihood = Eigen::MatrixXf::Zero(likelihood.rows(),
    // likelihood.cols()); logNegLikelihood =
    // Eigen::MatrixXf::Zero(likelihood.rows(), likelihood.cols());
    // negLikelihood = ones + (-1)* likelihood;
    // #pragma omp parallel for collapse(2)
    // for (int row = 0; row < likelihood.rows(); row++){
    //   for (int col = 0; col < likelihood.cols(); col++){
    //     // Ignore cells belonging to obstacles
    //     index(0) = row;
    //     index(1) = col;
    //     _rfid_belief_maps.getPosition(index, point);
    //     if (_rfid_belief_maps.atPosition("ref_map", point) ==
    //     _free_space_val){
    //       tmp = log(likelihood(row, col));
    //       if (isinf(tmp))
    //         tmp = 0.0;
    //       logLikelihood(row, col) = tmp;
    //       tmp = log(negLikelihood(row, col));
    //       if (isinf(tmp))
    //         tmp = 0.0;
    //       logNegLikelihood(row, col) = tmp;
    //     }
    //   }
    // }

    totalEntropy += (-likelihood.cwiseProduct(logLikelihood) -
                     negLikelihood.cwiseProduct(logNegLikelihood)).sum();
  }
  // std::chrono::steady_clock::time_point end =
  // std::chrono::steady_clock::now(); std::chrono::duration<double> time_span =
  // std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
  // std::cout << time_span.count() << " [secs]" << std::endl;
  // cout << "H:" << totalEntropy << endl;
  return totalEntropy;
}

double RadarModel::getTotalEntropy(double x, double y, double orientation,
                                   double size_x, double size_y, int tag_i) {
  // TODO: I'm not using the orientation. Maybe it would be better to use a
  // polygon iterator,
  //     so we can rotate edges around the center and have a more flexible thing

  // submapStartIndex the start index of the submap, typically top-left index.
  grid_map::Index submapStartIndex, submapEndIndex, submapBufferSize;
  grid_map::Position submapStartPosition(x + (size_x / 2), y + (size_y / 2));
  grid_map::Position submapEndPosition(x - (size_x / 2), y - (size_y / 2));

  if (!_rfid_belief_maps.getIndex(submapStartPosition, submapStartIndex)) {
    submapStartIndex = grid_map::Index(0, 0);
    // std::cout<<"Clip start!" << std::endl;
  }

  if (!_rfid_belief_maps.getIndex(submapEndPosition, submapEndIndex)) {
    Size siz = _rfid_belief_maps.getSize();
    submapEndIndex = grid_map::Index(siz(0) - 1, siz(1) - 1);
    // std::cout<<"Clip end!" << std::endl;
  }

  submapBufferSize = submapEndIndex - submapStartIndex;

  grid_map::SubmapIterator iterator(_rfid_belief_maps, submapStartIndex,
                                    submapBufferSize);
  return getTotalEntropy(x, y, orientation, iterator, tag_i);
}

double RadarModel::getTotalEntropy(double x, double y, double orientation,
                                   grid_map::SubmapIterator iterator,
                                   int tag_i) {

  double total_entropy;
  Position point;
  double likelihood, neg_likelihood, log2_likelihood, log2_neg_likelihood = 0.0;

  std::string tagLayerName = getTagLayerName(tag_i);

  total_entropy = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        likelihood = _rfid_belief_maps.atPosition(tagLayerName, point);
        if (isnan(likelihood))
          likelihood = 0.0;
        neg_likelihood = 1 - likelihood;
        if (isnan(neg_likelihood))
          neg_likelihood = 0.0;

        log2_likelihood = log2(likelihood);
        if (isinf(log2_likelihood))
          log2_likelihood = 0.0;
        log2_neg_likelihood = log2(neg_likelihood);
        if (isinf(log2_neg_likelihood))
          log2_neg_likelihood = 0.0;
        // cout << " l: " << log2_likelihood << endl;
        // likelihood =
        // rfid_tools->rm.getBeliefMaps().atPosition(layerName,rel_point);
        total_entropy += -likelihood * log2_likelihood -
                         neg_likelihood * log2_neg_likelihood;
      }
    }
  }
  return total_entropy;
}

double RadarModel::getTotalEntropyEllipse(Pose target, double maxX, double minX, int tag_i){
                                     //1.-  Get elipsoid iterator.
  // Antenna is at one of the focus of the ellipse with center at antennaX, antennaY, tilted antennaHeading .
  // http://www.softschools.com/math/calculus/finding_the_foci_of_an_ellipse/
  // if a is mayor axis and b is minor axis
  // a-c= minX
  // a+c= maxX
  // a = (maxX + minX)/2
  // c  = maxX/2 + minX
  // b  = sqrt(a^2-c^2)

  // mirror y axis!!!!
  double antennaX = target.getX();
  double antennaY = target.getY();
  double antennaHeading = target.getOrientation() * 3.14/180;

  double a =  (abs(maxX) + abs(minX))/2.0;
  double c =  (abs(maxX) - abs(minX))/2;
  double b = sqrt((a*a)-(c*c));
  double xc = antennaX + (c*cos(antennaHeading));
  double yc = antennaY + (c*sin(antennaHeading));

  Position center(xc, yc); // meters
  Length length(2*a, 2*b);
  grid_map::EllipseIterator el_iterator(_rfid_belief_maps, center, length, antennaHeading);
  return getTotalEntropyEllipse(target, el_iterator, tag_i);
}

double RadarModel::getTotalEntropyEllipse(Pose target, grid_map::EllipseIterator iterator,
                                   int tag_i) {

  double total_entropy;
  Position point;
  double likelihood, neg_likelihood, log2_likelihood, log2_neg_likelihood = 0.0;

  std::string tagLayerName = getTagLayerName(tag_i);
  tagLayerName = "kl";

  total_entropy = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        likelihood = _rfid_belief_maps.atPosition(tagLayerName, point);
        if (isnan(likelihood))
          likelihood = 0.0;
        neg_likelihood = 1 - likelihood;
        if (isnan(neg_likelihood))
          neg_likelihood = 0.0;

        log2_likelihood = log2(likelihood);
        if (isinf(log2_likelihood))
          log2_likelihood = 0.0;
        log2_neg_likelihood = log2(neg_likelihood);
        if (isinf(log2_neg_likelihood))
          log2_neg_likelihood = 0.0;
        // cout << " l: " << log2_likelihood << endl;
        // likelihood =
        // rfid_tools->rm.getBeliefMaps().atPosition(layerName,rel_point);
        total_entropy += -likelihood * log2_likelihood -
                         neg_likelihood * log2_neg_likelihood;
      }
    }
  }
  return total_entropy;
}

void RadarModel::printEllipse(double x, double y, double orient_rad, double maxX, double minX){
  std::string fileURI;

  cout << "[ELLIPSE] x: " << x << ", y: " << y << ", orient_rad: " << orient_rad << endl;

  double a =  (abs(maxX) + abs(minX))/2.0;
  double c =  (abs(maxX) - abs(minX))/2;
  double b = sqrt((a*a)-(c*c));
  double xc = x + (c*cos(orient_rad));
  double yc = y + (c*sin(orient_rad));

 

  Position center(xc, yc);
  Length length(2*a, 2*b);
  grid_map::EllipseIterator iterator(_rfid_belief_maps, center, length, orient_rad);

 

  _rfid_belief_maps.add("ellipse_test", 0);  

 

  for (iterator; !iterator.isPastEnd(); ++iterator) {
   _rfid_belief_maps.at("ellipse_test",*iterator) = 1;
  }

 

  fileURI = "/tmp/testEllipse.png";
  getImage(&_rfid_belief_maps, "ellipse_test", fileURI);
}

double RadarModel::getTotalKL(double x, double y, double orientation,
                              double size_x, double size_y, int tag_i) {
  // TODO: I'm not using the orientation. Maybe it would be better to use a
  // polygon iterator,
  //     so we can rotate edges around the center and have a more flexible thing

  // submapStartIndex the start index of the submap, typically top-left index.
  grid_map::Index submapStartIndex, submapEndIndex, submapBufferSize;
  grid_map::Position submapStartPosition(x + (size_x / 2), y + (size_y / 2));
  grid_map::Position submapEndPosition(x - (size_x / 2), y - (size_y / 2));

  if (!_rfid_belief_maps.getIndex(submapStartPosition, submapStartIndex)) {
    submapStartIndex = grid_map::Index(0, 0);
    // std::cout<<"Clip start!" << std::endl;
  }

  if (!_rfid_belief_maps.getIndex(submapEndPosition, submapEndIndex)) {
    Size siz = _rfid_belief_maps.getSize();
    submapEndIndex = grid_map::Index(siz(0) - 1, siz(1) - 1);
    // std::cout<<"Clip end!" << std::endl;
  }

  submapBufferSize = submapEndIndex - submapStartIndex;

  grid_map::SubmapIterator iterator(_rfid_belief_maps, submapStartIndex,
                                    submapBufferSize);

  return getTotalKL(x, y, orientation, iterator, tag_i);
}

// Adapted from
// https://gitlab.math.ethz.ch/NumCSE/NumCSE/blob/3c723d06ffacab3dc45726ad0a95e33987dc35aa/Utils/meshgrid.hpp

//! Generates a mesh, just like Matlab's meshgrid
//  Template specialization for column vectors (Eigen::VectorXd)
//  in : x, y column vectors 
//       X, Y matrices, used to save the mesh
template <typename Scalar>
void RadarModel::meshgrid(const Eigen::Matrix<Scalar, -1, 1>& x, 
              const Eigen::Matrix<Scalar, -1, 1>& y,
              Eigen::Matrix<Scalar, -1, -1>& X,
              Eigen::Matrix<Scalar, -1, -1>& Y) {
  const long nx = x.size(), ny = y.size();
  X.resize(ny, nx);
  Y.resize(ny, nx);
  for (long i = 0; i < ny; ++i) {
    X.row(i) = x.transpose();
  }

  // 
  // for (long j = 0; j < nx; ++j) {
  //   Y.col(j) = y;
  // }
  for (long j = 0; j < nx; ++j) {
    Y.col(j) = y.reverse();
  }
}


//! Generates a mesh, just like Matlab's meshgrid
//  Template specialization for row vectors (Eigen::RowVectorXd)
//  in : x, y row vectors 
//       X, Y matrices, used to save the mesh
template <typename Scalar>
void RadarModel::meshgrid(const Eigen::Matrix<Scalar, 1, -1>& x, 
              const Eigen::Matrix<Scalar, 1, -1>& y,
              Eigen::Matrix<Scalar, -1, -1>& X,
              Eigen::Matrix<Scalar, -1, -1>& Y) {
  Eigen::Matrix<Scalar, -1, 1> xt = x.transpose(),
                               yt = y.transpose();
  meshgrid(xt, yt, X, Y);
}
double RadarModel::getTotalKL(double x, double y, double orientation,
                              grid_map::SubmapIterator iterator, int tag_i) {

  double total_KL, tmp_KL = 0.0;
  Position point;
  double prior, posterior = 0.0;

  std::string tagLayerName = getTagLayerName(tag_i);

  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        prior = _rfid_belief_maps.atPosition(tagLayerName, point);
        posterior = _rfid_belief_maps.atPosition("kl", point);
        tmp_KL = posterior * log(posterior / prior);
        if (isnan(tmp_KL))
          tmp_KL = 0;
        total_KL += tmp_KL;
        cout << " Prior: " << prior << endl;
        cout << " Posterior: " << posterior << endl;
        cout << " totalKL: " << total_KL << endl;
      }
    }
  }
  return total_KL;
}

void RadarModel::addTmpMeasurementRFIDCriterion(double x_m, double y_m,
                                                double orientation_deg,
                                                double rxPower, double phase,
                                                double freq, int i,
                                                double len_update) {

  // cout << "computeKL: " << computeKL << endl;
  double rel_x, rel_y, prob_val, orientation_rad;
  double glob_x, glob_y, delt_x, delt_y;
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;

  double prior, posterior, likelihood, bayes_num, bayes_den;
  Eigen::MatrixXf prob_matm, rxPw_mat, likl_mat;
  // std::string tagLayerName = getTagLayerName(i);

  
  // CORRECTION STEP
  // get the expected power at each point
  rxPw_mat = getFriisMat(x_m, y_m, orientation_deg, freq);

  // get the likelihood of the received power at each point
  likl_mat = getProbCond(rxPw_mat, rxPower, _sigma_power);

  // Where X_mat is < than SENSITIVITY, the tag wont be... prob 0
  // likl_mat = (likl_mat.array()<=SENSITIVITY).select(0,likl_mat);

  // this should remove prob at obstacles
  Eigen::MatrixXf obst_mat = _rfid_belief_maps["ref_map"];
  likl_mat = (obst_mat.array() == _free_space_val).select(likl_mat, 0);

  // normalize in this space:
  bayes_den = likl_mat.sum();
  if (bayes_den > 0) {
    likl_mat = likl_mat / bayes_den;
    // now do bayes ...  everywhere
    _rfid_belief_maps["kl"] = likl_mat;
  }

  // Normalize
  // double totalW = _rfid_belief_maps["kl"].sum();
  // if (totalW > 0) {
  //   _rfid_belief_maps["kl"] /= totalW;
  // }
  // cout << "KL[" << _counter << endl;
  // _counter++;

  // First we get the Probability distribution associated with (
  // rxPower,phase,freq) using our defined active area grids
  // if (rxPower > SENSITIVITY) {
  //   prob_mat = getPowProbCondRFIDCriterion(
  //       rxPower, freq); //.cwiseProduct(getPhaseProbCond(phase, freq));
  //                       // } else{
  //   //   prob_mat = getNegProb(getPowLayerName(freq), rxPower,
  //   //   _sigma_power);//.cwiseProduct(getPhaseProbCond(phase, freq));
  //   // //   if (i == 0){
  //   // //     cout << "Neg: " << prob_mat.sum() << endl;
  //   // //   }
  //   // }

  //   // We store this data matrix in a temporal layer
  //   this->createTempProbLayerRFIDCriterion(
  //       prob_mat, x_m, y_m, orientation_deg,
  //       len_update); // size of the activeArea

  //   // so we need to translate this matrix to robot pose and orientation
  //   orientation_rad = orientation_deg * M_PI / 180.0;
  //   // We ned to normalise the new probability over the entire grid, so
  //   // we need to multiple all the priors for all the new measurements
  //   // and this will be our denominator while applying Bayes
  //   // NB: posterior = likelihood * prior / normalizing_factor
  //   // where normalisizing_factor is a sum over all the grid of likelihood *
  //   // prior bayes_den = getNormalizingFactorBayesRFIDActiveArea(x_m, y_m,
  //   // orientation_rad, tagLayerName);  // computed on the entire activeArea
  //   // if (bayes_den != 0.0 and !isnan(bayes_den)){
  //   // Now we can proceed with the update
  //   update_edges = getSubMapEdges(
  //       x_m, y_m, orientation_rad,
  //       len_update); // NOTE: update computed only on a small subset
  //   for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges);
  //        !iterator.isPastEnd(); ++iterator) {
  //     // check if point is an obstacle:
  //     if (_rfid_belief_maps.at("ref_map", *iterator) == _free_space_val) {
  //       // get the relative point
  //       _rfid_belief_maps.getPosition(*iterator, glob_point);
  //       // rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);
        
  //       // We probably don't need this anymore after dropping activemap
  //       // rel_point = getSubMapRelPoint(glob_point, x_m, y_m, orientation_rad,
  //       //                               len_update);
  //       likelihood =
  //           _tmp_rfid_c_map.atPosition("temp", rel_point); // the measurement
  //       prior = _rfid_belief_maps.at(tagLayerName,
  //                                    *iterator); // the value in the map
  //       bayes_num = prior * likelihood;
  //       // posterior = bayes_num / bayes_den;
  //       _rfid_belief_maps.at("kl", *iterator) = bayes_num;
  //     } else {
  //       // this shouldn't be necessary ....
  //       _rfid_belief_maps.at(tagLayerName, *iterator) = 0;
  //     }
  //   }
  //   // }
  // }
}

Eigen::MatrixXf RadarModel::getPowProbCondRFIDCriterion(double rxPw,
                                                        double f_i) {
  // probability of receiving less than X1 in every cell
  Eigen::MatrixXf x1_mat = getNegProbRFIDCriterion(rxPw, _sigma_power);
  // std::cout << "x1_mat: " << x1_mat.sum()/(siz(0)*siz(1)) << endl;

  // probability of receiving between X1 and X2 in every cell
  Eigen::MatrixXf ans = (x1_mat).array().abs();

  return ans;
}

void RadarModel::createTempProbLayerRFIDCriterion(Eigen::MatrixXf prob_mat,
                                                  double x_m, double y_m,
                                                  double orientation_deg,
                                                  double len_update) {
  double orientation_rad;
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;

  // We store this data matrix in a temporal layer
  // Here, boundaries are relative to position (0,0,0º): p1 (nx/2,ny/2) ,
  // pcreateTempProbLayerRFIDCriterion2 (-nx/2, ny/2), p3 (-nx/2,-ny/2), p4
  // (nx/2,-ny/2)
  _tmp_rfid_c_map.add("temp", prob_mat);
  // now we remove from this layer probabilities inside obstacle cells
  // so we need to translate this matrix to robot pose and orientation
  orientation_rad = orientation_deg * M_PI / 180.0;
  update_edges = getSubMapEdges(x_m, y_m, orientation_rad, len_update);

  for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges);
       !iterator.isPastEnd(); ++iterator) {
    // check if it's an obstacle:
    if (_rfid_belief_maps.at("ref_map", *iterator) != _free_space_val) {
      _rfid_belief_maps.getPosition(*iterator, glob_point);
      // We don't need this anymore without active area
      // rel_point =
      //     getSubMapRelPoint(glob_point, x_m, y_m, orientation_rad, len_update);
      // remove prob in obstacles
      _tmp_rfid_c_map.atPosition("temp", rel_point) = 0.0;
    }
  }
}

Eigen::MatrixXf RadarModel::getNegProbRFIDCriterion(double x, double sigm) {
  Eigen::MatrixXf friis_mat = _tmp_rfid_c_map["temp"];
  // gaussian cdp
  // cdp(x,mu,sigma) = 0.5 * ( 1 + erf( (x - mu) / (sigma * sqrt(2)) )
  Eigen::MatrixXf erf_mat = (x - friis_mat.array()) / (sigm * sqrt(2.0));
  erf_mat = 0.5 + 0.5 * erf_mat.array().erf();
  return erf_mat;
}

grid_map::Polygon RadarModel::getSubMapEdges(double robot_x, double robot_y,
                                             double robot_head, double len) {
  grid_map::Polygon polygon;

  // trying not to replicate code, I'm using the cv points one that works
  cv::Point cvrobot = getPoint(robot_x, robot_y);
  cv::Point pts[4];
  pts[0] = getPoint(robot_x - len / 2, robot_y - len / 2);
  pts[1] = getPoint(robot_x - len / 2, robot_y + len / 2);
  pts[2] = getPoint(robot_x + len / 2, robot_y + len / 2);
  pts[3] = getPoint(robot_x + len / 2, robot_y - len / 2);

  rotatePoints(pts, 4, cvrobot.x, cvrobot.y, robot_head);

  // cast to gridmap points and add to polygon
  polygon.setFrameId(_rfid_belief_maps.getFrameId());
  for (int i = 0; i < 4; i++) {
    polygon.addVertex(fromPoint(pts[i]));
  }

  return polygon;
}

// Position RadarModel::getSubMapRelPoint(Position glob_point, double x_m,
//                                        double y_m, double orientation_rad,
//                                        double len) {
//   double rel_x, rel_y;
//   Position rel_point;
//   // we may need these for castings ...
//   grid_map::Position orig = _active_area_maps.getPosition();
//   double min_x = orig(0) - len / 2 + _resolution;
//   double max_x = orig(0) + len / 2 - _resolution;
//   double min_y = orig(1) - len / 2 + _resolution;
//   double max_y = orig(1) + len / 2 - _resolution;

//   rel_x = (glob_point.x() - x_m) * cos(orientation_rad) +
//           (glob_point.y() - y_m) * sin(orientation_rad);
//   rel_y = -(glob_point.x() - x_m) * sin(orientation_rad) +
//           (glob_point.y() - y_m) * cos(orientation_rad);
//   rel_point = Position(rel_x, rel_y);

//   // because of rounding, we may have to adjust this
//   if (!_active_area_maps.isInside(rel_point)) {
//     rel_x = std::min(std::max(rel_x, min_x), max_x);
//     rel_y = std::min(std::max(rel_y, min_y), max_y);
//     rel_point = Position(rel_x, rel_y);
//   }
//   return rel_point;
// }

void RadarModel::updateTagsPosition(std::vector<std::pair<double,double>> tags_coord){
  _tags_coords = tags_coord;
}

Eigen::MatrixXf RadarModel::getGaussianKernel(int rows, int cols, int x_mean, int y_mean, double x_sigma, double y_sigma)
{
    // const auto y_mid = (rows-1) / 2.0;
    // const auto x_mid = (cols-1) / 2.0;

    const auto y_mid = y_mean;
    const auto x_mid = x_mean;

    const auto x_spread = 1. / (x_sigma*x_sigma*2);
    const auto y_spread = 1. / (y_sigma*y_sigma*2);

    const auto denominator = 8 * std::atan(1) * x_sigma * y_sigma;

    std::vector<double> gauss_x, gauss_y;

    gauss_x.reserve(cols);
    for (auto i = 0;  i < cols;  ++i) {
        auto x = i - x_mid;
        gauss_x.push_back(std::exp(-x*x * x_spread));
    }

    gauss_y.reserve(rows);
    for (auto i = 0;  i < rows;  ++i) {
        auto y = i - y_mid;
        gauss_y.push_back(std::exp(-y*y * y_spread));
    }

    Eigen::MatrixXf kernel = Eigen::MatrixXf::Zero(rows, cols);
    for (auto j = 0;  j < rows;  ++j)
        for (auto i = 0;  i < cols;  ++i) {
            kernel(j,i) = gauss_x[i] * gauss_y[j] / denominator;
        }

    return kernel;
}

Eigen::MatrixXf RadarModel::getGaussianRandomWalk( std::pair<int, int> mean, int tag_id){
  
  double sigma = 0.5;
  // Increase the variance at every iteration
  sigma = _iteration_no_readings[tag_id]*sigma;
  // NOTE: Eigen follows a different coordinate system, so we need to swap row and cols 
  Eigen::MatrixXf kernel = getGaussianKernel( _Nrow,
                                              _Ncol,
                                              _rfid_belief_maps.getLength().y() - mean.second, //58
                                              _rfid_belief_maps.getLength().x() - mean.first,  //37
                                             sigma, sigma);
                                             
  kernel = kernel/kernel.maxCoeff();
  // cout << "   I: "<<_iteration_no_readings << ", sigma: " << sigma << endl;;

  
  // cout << "     " << mean.first << "," << mean.second << endl;

  // Show image in a window but first normalize in 0-255
  // double minValue, maxValue;
  // kernel = kernel  / kernel.maxCoeff();
  // kernel = 255*kernel;
  // // Naming the window 
  // cv::String geek_window = "MY SAVED IMAGE"; 
  // // Creating a window 
  // cv::namedWindow(geek_window); 
  // cv::Mat kernel_mat = cv::Mat::zeros(_Nrow, _Ncol, CV_32F);
  // cv::eigen2cv(kernel, kernel_mat);
  // Showing the image in the defined window 
    // cv::imshow(geek_window, kernel_mat); 
    // // waiting for any key to be pressed 
    // cv::waitKey(0); 
    // // destroying the creating window 
    // cv::destroyWindow(geek_window); 
  

  return kernel;

}