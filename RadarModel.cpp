#include "RadarModel.hpp"
#include <unsupported/Eigen/SpecialFunctions>

using namespace std;
using namespace grid_map;


        SplineFunction::SplineFunction()
        {}

        SplineFunction::SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec)
          : x_min(x_vec.minCoeff()),
            x_max(x_vec.maxCoeff()),
            y_min(y_vec.minCoeff()),
            y_max(y_vec.maxCoeff()),
            // Spline fitting here. X values are scaled down to [0, 1] for this.
            spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 6), scaled_values(x_vec)))  // No more than cubic spline, but accept short vectors.
        { 

            // for(int i=0; i< x_vec.size(); i++){
            //       cout <<"G(" << x_vec[i] << ")= "<< interpDeg(x_vec[i]) << std::endl;
            // } 

        }
        
        // x values need to be scaled down in extraction as well.
        double SplineFunction::interpDeg(double x) const {      
          double y;
          y = spline_(scaled_value(x))(0);


//          if ((y>y_max) || (y<y_min)){
//                  cout <<"Angle " << x << " produced gain "<< y << std::endl;
//          }

          // interpolation may produce values bigger and lower than our limits ...
          
          y = max(min(y, y_max), y_min );
          return y;
        }

        double SplineFunction::interpRad(double x) const {
            return interpDeg(x*180.0/M_PI); 
        }
      
        // Helpers to scale X values down to [0, 1]
        double SplineFunction::scaled_value(double x) const {
          return (x - x_min) / (x_max - x_min);
        }

        Eigen::RowVectorXd SplineFunction::scaled_values(Eigen::VectorXd const &x_vec) const {
          return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
        }

//////////////////


RadarModel::RadarModel(){};

RadarModel::RadarModel(const double nx, const double ny, const double resolution, const double sigma_power, const double sigma_phase, const double txtPower, const std::vector<double> freqs, const std::vector<std::pair<double,double>> tags_coords, const std::string imageFileURI, std::string model ) {
        _model = model;
        _sizeXaa = nx;
        _sizeYaa = ny;
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
        //cout <<"antenna gain spline model built" << std::endl;

        // build active area grid
        // active area grid goes from -X/2,-Y/2 to X/2,Y/2 with resolution
        _active_area_maps.setGeometry(Length(_sizeXaa,_sizeYaa), _resolution);
        _tmp_rfid_c_map.setGeometry(Length(2,2), _resolution);
        _tmp_rfid_c_map.add("temp", 0.0);

        // This equals to a Size
        Size siz = _active_area_maps.getSize();
      
        // Create map of estimated received powers at reference each frequency in active area
        Position point;
        double tag_x, tag_y, tag_r, tag_h,phi, rxP;
        Index ind;
        std::string layerName;

        for(std::vector<double>::size_type i = 0; i != _freqs.size(); i++) {
              
              Eigen::MatrixXf rxPw_mat = Eigen::MatrixXf(siz(0), siz(1));
              Eigen::MatrixXf delay_mat = Eigen::MatrixXf(siz(0), siz(1));
              fillFriisMat( &rxPw_mat,  &delay_mat, freqs[i], 0 );
            

              // we have like an mean power, phase values map for each frequency and position with this power and setup ...
              layerName = getPowLayerName(freqs[i]);
              _active_area_maps.add(layerName, rxPw_mat);
              layerName = getPhaseLayerName(freqs[i]);
              _active_area_maps.add(layerName, delay_mat);

              // // ...................................
              fillFriisMat( &rxPw_mat,  &delay_mat, freqs[i], 0.5*_resolution );
              layerName = getPowLayerName(freqs[i])+"_low";
              _active_area_maps.add(layerName, rxPw_mat);
              layerName = getPhaseLayerName(freqs[i])+"_low";
              _active_area_maps.add(layerName, delay_mat);

              fillFriisMat( &rxPw_mat,  &delay_mat, freqs[i], -0.5*_resolution );
              layerName = getPowLayerName(freqs[i])+"_hig";
              _active_area_maps.add(layerName, rxPw_mat);
              layerName = getPhaseLayerName(freqs[i])+"_hig";
              _active_area_maps.add(layerName, delay_mat);
              // // ...................................
              

              // cout <<"Models for F "<< getLayerName(freqs[i]) << " built" << std::endl;
        }

        // rfid beliefs global map: One layer per tag
        for(int i = 0; i <_numTags; ++i) {
           layerName = getTagLayerName(i);
           _rfid_belief_maps.add(layerName, 0.5);  // the cells need to have a uniform distribution at the beginning
          // // int count = 0;
          // // Parse the map and set to 0 cells with obstacles
          // for (grid_map::GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd(); ++iterator) {                  
          //   // get cell center of the cell in the map frame.            
          //   _rfid_belief_maps.getPosition(*iterator, point);
          //   if(_rfid_belief_maps.atPosition("ref_map",point)!= _free_space_val){
          //     _rfid_belief_maps.at(layerName,*iterator) = 0.0;
          //     // count++;
          //   }               
          // }

          // // now we renormalize the  map
          // double totalW = _rfid_belief_maps[layerName].sum();
          // cout << "SUM: " << totalW << endl;
          // cout << "Max: " << _rfid_belief_maps[layerName].maxCoeff() << endl;
          // cout << "min: " << _rfid_belief_maps[layerName].minCoeff() << endl;
          // // cout << "#Obs: " << count << endl;

          // if (totalW > 0){
          //   _rfid_belief_maps[layerName] = _rfid_belief_maps[layerName]/totalW;
          // }
          // // cout << "Max: " << _rfid_belief_maps[layerName].maxCoeff() << endl;
          // cout << "min: " << _rfid_belief_maps[layerName].minCoeff() << endl;
        }
        clearObstacleCellsRFIDMap();
        normalizeRFIDMap();


        debugInfo();




    }


    void RadarModel::fillFriisMat(Eigen::MatrixXf *rxPw_mat, Eigen::MatrixXf *delay_mat, double freq_i, double radius_offset ){
              Index ind;
              Position point;
              double tag_x, tag_y, tag_r, tag_h,phi, rxP;

              for (grid_map::GridMapIterator iterator(_active_area_maps); !iterator.isPastEnd(); ++iterator) {
                  // matrix indexes...
                  ind = *iterator;
                  
                  // get cell center of the cell in the map frame.            
                  _active_area_maps.getPosition(ind, point);
                  // that is where the tag is.
                  tag_x = point.x();
                  tag_y = point.y();

                  getSphericCoords(tag_x,tag_y, tag_r, tag_h);

                  // PHASE_CONSTANT  === 4*pi/c 
                  phi = fmod ( PHASE_CONSTANT * freq_i * tag_r, M_PI);

                  rxP = received_power_friis_polar(tag_r + radius_offset, tag_h,freq_i, _txtPower, _antenna_gains);
                  //cout <<"rxPw: ("<< rxP <<")" << std::endl;
                  (*rxPw_mat)(ind(0),ind(1))  = rxP;                  
                  (*delay_mat)(ind(0),ind(1))  = phi;                  
              }

            }



void RadarModel::initRefMap(const std::string imageURI){
        std::cout<<"\nIniting Ref map."  <<std::endl;

        cv::Mat _imageCV = cv::imread(imageURI , CV_LOAD_IMAGE_UNCHANGED );
        // this alligns image with our coordinate systems
        cv::flip(_imageCV, _imageCV, -1);

        _Ncol = _imageCV.cols; // radar model total x-range space (cells).
        _Nrow = _imageCV.rows; // radar model total y-range space (cells).

        // cell value ranges
        double  minValue;
        double  maxValue;

        double orig_x,len_x, orig_y,len_y;
        
        len_x= _Nrow * _resolution;
        len_y= _Ncol * _resolution;
        //2D position of the grid map in the grid map frame [m].
        // orig will be placed at the UPPER LEFT CORNER THE IMAGE, thus all coordinates will be positive
        
        orig_x= ( _Nrow - 1 ) * (_resolution / 2.0) ;
        orig_y= ( _Ncol - 1 ) * (_resolution / 2.0) ;

        _rfid_belief_maps = grid_map::GridMap (vector<string>({"ref_map"}));
        _rfid_belief_maps.setGeometry(Length(len_x, len_y), _resolution, Position(orig_x, orig_y));

        cv::minMaxLoc(_imageCV, &minValue, &maxValue);
        _free_space_val = maxValue;
        _rfid_belief_maps["ref_map"].setConstant(NAN);
        GridMapCvConverter::addLayerFromImage<unsigned char, 3>(_imageCV, "ref_map", _rfid_belief_maps, minValue, maxValue);
        
        std::cout << " Input map has " <<   _rfid_belief_maps.getSize()(1) << " cols by " <<  _rfid_belief_maps.getSize()(0) <<" rows "  <<std::endl;
        std::cout << " Orig at: (" << orig_x << ", " << orig_y<<") m. " <<std::endl;
        std::cout << " Size: (" << _rfid_belief_maps.getLength().x() << ", " << _rfid_belief_maps.getLength().y() <<") m. " <<std::endl;
        std::cout << " Values range: (" << minValue << ", " << maxValue <<")   " <<std::endl;
        std::cout << " Using : (" << maxValue <<") value as free space value  " <<std::endl;

        grid_map::Position p;       
        grid_map::Index index;

        // std::cout<<"\nTesting boundaries: " <<std::endl;
        index =grid_map::Index(0,0);
        // std::cout<<"P: Index("  << 0 << ", " << 0 << ") " <<std::endl;
        if (_rfid_belief_maps.getPosition(index,p)){  
            // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }

        index =grid_map::Index(_Nrow-1,0);
        // std::cout<<"P: Index("  << (_Nrow-1) << ", " << 0 << ") " <<std::endl;
        if (_rfid_belief_maps.getPosition(index,p)){  
            // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }

        index =grid_map::Index(0,_Ncol-1);
        // std::cout<<"P: Index("  << (0) << ", " << (_Ncol-1) << ") " <<std::endl;
        if (_rfid_belief_maps.getPosition(index,p)){  
            // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }

        index =grid_map::Index(_Nrow-1,_Ncol-1);
        // std::cout << "P: Index("  << (_Nrow-1) << ", " << (_Ncol-1) << ") " <<std::endl;        
        if (_rfid_belief_maps.getPosition(index,p)){  
            // std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          // std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }
        // std::cout << "............................. " << std::endl << std::endl;        
}

//So, robot at pr (x,y,orientation) (long, long, int) receives rxPower,phase,freq from tag i . 
//TODO: this is a simplistic model that just "ignores" walls: but they do have absortion and reduce received power at their locations...
void RadarModel::addMeasurement(double x_m, double y_m, double orientation_deg, double rxPower, double phase, double freq, int i){
  
  // cout << "computeKL: " << computeKL << endl;
  double rel_x, rel_y, prob_val, orientation_rad;
  double glob_x, glob_y, delt_x, delt_y;
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;

  double prior, posterior, likelihood, bayes_num, bayes_den;
  Eigen::MatrixXf prob_mat;
  std::string tagLayerName = getTagLayerName(i);
  // First we get the Probability distribution associated with ( rxPower,phase,freq) using our defined active area grids
  if (rxPower > SENSITIVITY){
    prob_mat = getPowProbCond(rxPower, freq);  //.cwiseProduct(getPhaseProbCond(phase, freq));
  // } else{
  //   prob_mat = getNegProb(getPowLayerName(freq), rxPower, _sigma_power);  //.cwiseProduct(getPhaseProbCond(phase, freq));
  // }
  

  // We store this data matrix in a temporal layer
  createTempProbLayer(prob_mat, x_m, y_m, orientation_deg);

  // so we need to translate this matrix to robot pose and orientation
  orientation_rad = orientation_deg * M_PI/180.0;
  // We ned to normalise the new probability over the entire grid, so
  // we need to multiple all the priors for all the new measurements
  // and this will be our denominator while applying Bayes
  // NB: posterior = likelihood * prior / normalizing_factor
  // where normalisizing_factor is a sum over all the grid of likelihood * prior
  bayes_den = getNormalizingFactorBayesRFIDActiveArea(x_m, y_m, orientation_rad, tagLayerName);

  if (bayes_den != 0.0 and !isnan(bayes_den)){
    // Now we can proceed with the update
    update_edges = getActiveMapEdges(x_m, y_m, orientation_rad);
    for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges); !iterator.isPastEnd(); ++iterator){
          // check if point is an obstacle:                      
          if (_rfid_belief_maps.at("ref_map",*iterator) == _free_space_val){
            // get the relative point
            _rfid_belief_maps.getPosition(*iterator, glob_point);                  
            rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);
            // get the (measurement) prob :
            likelihood = _active_area_maps.atPosition("temp",rel_point);  // the measurement
            prior = _rfid_belief_maps.at(tagLayerName,*iterator);  // the value in the 
            // Update the belief only if the measurement or the prior is different from 0
            bayes_num = prior * likelihood;
            posterior = bayes_num / bayes_den;
            _rfid_belief_maps.at(tagLayerName,*iterator) = posterior;
        } else {
          // this shouldn't be necessary ....
          _rfid_belief_maps.at(tagLayerName,*iterator) = 0;
        }
      
    }
  }
  normalizeRFIDLayer(tagLayerName);
  }
}

void RadarModel::addTmpMeasurementRFIDCriterion(double x_m, double y_m, double orientation_deg, double rxPower, double phase, double freq, int i, double len_update){
  
  // cout << "computeKL: " << computeKL << endl;
  double rel_x, rel_y, prob_val, orientation_rad;
  double glob_x, glob_y, delt_x, delt_y;
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;

  double prior, posterior, likelihood, bayes_num, bayes_den;
  Eigen::MatrixXf prob_mat;
  std::string tagLayerName = getTagLayerName(i);

  _rfid_belief_maps.add("kl", 0.0);

  // First we get the Probability distribution associated with ( rxPower,phase,freq) using our defined active area grids
  if (rxPower > SENSITIVITY){
    prob_mat = getPowProbCondRFIDCriterion(rxPower, freq);//.cwiseProduct(getPhaseProbCond(phase, freq));
  // } else{
  //   prob_mat = getNegProb(getPowLayerName(freq), rxPower, _sigma_power);//.cwiseProduct(getPhaseProbCond(phase, freq));
  // //   if (i == 0){
  // //     cout << "Neg: " << prob_mat.sum() << endl;
  // //   }
  // }
  

    // We store this data matrix in a temporal layer
    this->createTempProbLayerRFIDCriterion(prob_mat, x_m, y_m, orientation_deg, len_update);  // size of the activeArea

    // so we need to translate this matrix to robot pose and orientation
    orientation_rad = orientation_deg * M_PI/180.0;
    // We ned to normalise the new probability over the entire grid, so
    // we need to multiple all the priors for all the new measurements
    // and this will be our denominator while applying Bayes
    // NB: posterior = likelihood * prior / normalizing_factor
    // where normalisizing_factor is a sum over all the grid of likelihood * prior
    // bayes_den = getNormalizingFactorBayesRFIDActiveArea(x_m, y_m, orientation_rad, tagLayerName);  // computed on the entire activeArea
    cout << "hey5" << endl;
    // if (bayes_den != 0.0 and !isnan(bayes_den)){
      // Now we can proceed with the update
      update_edges = getSubMapEdges(x_m, y_m, orientation_rad, len_update);  // NOTE: update computed only on a small subset
      for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges); !iterator.isPastEnd(); ++iterator){
            // check if point is an obstacle:                      
            if (_rfid_belief_maps.at("ref_map",*iterator) == _free_space_val){
              // get the relative point
              _rfid_belief_maps.getPosition(*iterator, glob_point);                  
              // rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);  
              rel_point = getSubMapRelPoint(glob_point, x_m, y_m, orientation_rad, len_update);  
              // get the (measurement) prob :
              // likelihood = _active_area_maps.atPosition("temp",rel_point);  // the measurement
              cout << "hey6" << endl;
              likelihood = _tmp_rfid_c_map.atPosition("temp",rel_point);  // the measurement
              cout << "hey7" << endl;
              prior = _rfid_belief_maps.at(tagLayerName,*iterator);  // the value in the map
              bayes_num = prior * likelihood;
              // posterior = bayes_num / bayes_den;
              _rfid_belief_maps.at("kl",*iterator) = bayes_num;
          } else {
            // this shouldn't be necessary ....
            _rfid_belief_maps.at(tagLayerName,*iterator) = 0;
          }
        
      }
    // }
  }
  
}

    void  RadarModel::saveProbMaps(std::string savePath){
        std::string tagLayerName;
        std::string fileURI;
        Eigen::MatrixXf data_mat ;

        //PrintMap(savePath);


        // prob distribution maps
        for(int i = 0; i <_numTags; ++i) {
          tagLayerName = getTagLayerName(i);
          // std::cout << " Saving layer [" << tagLayerName << "]" << std::endl ;
          fileURI  = savePath + "final_prob_" +tagLayerName + ".png";
          getImage(&_rfid_belief_maps, tagLayerName, fileURI);
          //data_mat = _rfid_belief_maps[tagLayerName];
          //PrintProb(fileURI, &data_mat, _Ncol*_resolution, _Nrow*_resolution, _resolution);
        }
    }

    //DEPRECATED ! print original map with tag locations
    void RadarModel::PrintMap( std::string savePath){
        std::string fileURI;
        cv::Mat image;
        grid_map::Index index;            
        cv::Scalar green( 0, 255, 0 );
        cv::Scalar blue( 255, 0, 0 );      
        cv::Scalar red( 0, 0, 255 );
        cv::Scalar yellow( 0, 255, 255 );      
        cv::Scalar purple( 128, 0, 128 );

        fileURI  = savePath + "ref_map.png";


        
        // Convert to image.
        const float minValue = _rfid_belief_maps["ref_map"].minCoeff();
        const float maxValue = _rfid_belief_maps["ref_map"].maxCoeff();

        GridMapCvConverter::toImage<unsigned char, 3>(_rfid_belief_maps, "ref_map", CV_8UC3, minValue, maxValue, image);
      
        //add tag locations into cv mat:              
        cv::Point center;

        for (int i = 0; i < _tags_coords.size(); i++){
            double x = _tags_coords[i].first;
            double y =  _tags_coords[i].second;
            grid_map::Position p(x,  y);                    
            _rfid_belief_maps.getIndex(p,index);         
            std::cout<<"Tag: (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;
                           
            center = cv::Point( index.x(), index.y() );
            //cv::circle( image,center,6,green,-1,8 );
            cv::circle(image, center , 5, purple, -1);

        }
        // map references: ..............................................................
        double maxX=  _rfid_belief_maps.getLength().x()/2;
        double maxY=  _rfid_belief_maps.getLength().y()/2;

        grid_map::Position p(0,  0);
        _rfid_belief_maps.getIndex(p,index);      
        cv::Point gree( index.y(), index.x() );
        std::cout<<"Green: (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

        p = Position(maxX,  -maxY);
        _rfid_belief_maps.getIndex(p,index);      
        cv::Point blu( index.y(), index.x() );
        std::cout<< "Blue (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

        p = Position(-maxX,  -maxY);
        _rfid_belief_maps.getIndex(p,index);  
        cv::Point re( index.y(), index.x() );
        std::cout<<"Red (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

        p = Position(-maxX,  maxY);
        _rfid_belief_maps.getIndex(p,index);  
        cv::Point yell( index.y(), index.x() );
        std::cout<<"Yellow (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;
        
        cv::circle(image, gree , 20, green, -1);
        cv::circle(image, blu , 20, blue, -1);
        cv::circle(image, re ,  20, red, -1);
        cv::circle(image, yell , 20, yellow, -1);







        // CV: Rotate 90 Degrees Clockwise To get our images to have increasing X to right and increasing Y up
        cv::transpose(image, image);
        cv::flip(image, image, 1);
        cv::imwrite( fileURI, image );
    
    }



    Eigen::MatrixXf  RadarModel::getPowProbCond(double rxPw, double f_i){
      std::string layerName = getPowLayerName(f_i);
      // Eigen::MatrixXf ans = getProbCond(layerName, rxPw, _sigma_power);
      Eigen::MatrixXf ans = getIntervProb(layerName, rxPw, _sigma_power);
      
      return ans;
    }

    Eigen::MatrixXf  RadarModel::getPowProbCondRFIDCriterion(double rxPw, double f_i){
      // probability of receiving less than X1 in every cell
      Eigen::MatrixXf x1_mat = getNegProbRFIDCriterion( rxPw, _sigma_power);
      //std::cout << "x1_mat: " << x1_mat.sum()/(siz(0)*siz(1)) << endl;

      // probability of receiving between X1 and X2 in every cell
      Eigen::MatrixXf ans = (x1_mat).array().abs();
      
      return ans;
    }

    Eigen::MatrixXf  RadarModel::getProbCond(std::string layer_i, double x, double sig){
      Eigen::MatrixXf ans;
      if (_model=="gaussian"){
          ans = getProbCondG(layer_i, x, sig);
      }else{
          ans = getProbCondLogN(layer_i, x, sig);
      }

      return ans;
    }

    void RadarModel::createTempProbLayer(Eigen::MatrixXf prob_mat, double x_m, double y_m, double orientation_deg) {
      double orientation_rad;
      Position rel_point, glob_point;
      grid_map::Polygon update_edges;

      // We store this data matrix in a temporal layer
      // Here, boundaries are relative to position (0,0,0º): p1 (nx/2,ny/2) , p2 (-nx/2, ny/2), p3 (-nx/2,-ny/2), p4 (nx/2,-ny/2) 
      _active_area_maps.add("temp", prob_mat);     

      // now we remove from this layer probabilities inside obstacle cells
      // so we need to translate this matrix to robot pose and orientation
      orientation_rad = orientation_deg * M_PI/180.0;
      update_edges = getActiveMapEdges(x_m, y_m, orientation_rad);

      for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges); !iterator.isPastEnd(); ++iterator){
          // check if it's an obstacle:                      
            if (_rfid_belief_maps.at("ref_map",*iterator) != _free_space_val){
                _rfid_belief_maps.getPosition(*iterator, glob_point);                  
                rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);
              // remove prob in obstacles
              _active_area_maps.atPosition("temp",rel_point) = 0.0;
            }
      }
    }

    void RadarModel::createTempProbLayerRFIDCriterion(Eigen::MatrixXf prob_mat, double x_m, double y_m, double orientation_deg, double len_update) {
      double orientation_rad;
      Position rel_point, glob_point;
      grid_map::Polygon update_edges;

      // We store this data matrix in a temporal layer
      // Here, boundaries are relative to position (0,0,0º): p1 (nx/2,ny/2) , pcreateTempProbLayerRFIDCriterion2 (-nx/2, ny/2), p3 (-nx/2,-ny/2), p4 (nx/2,-ny/2) 
      cout << "hey" << endl;
      _tmp_rfid_c_map.add("temp", prob_mat);     
      cout << "hey2" << endl;
      // now we remove from this layer probabilities inside obstacle cells
      // so we need to translate this matrix to robot pose and orientation
      orientation_rad = orientation_deg * M_PI/180.0;
      update_edges = getSubMapEdges(x_m, y_m, orientation_rad, len_update);
      cout << "hey3" << endl;

      for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges); !iterator.isPastEnd(); ++iterator){
        // check if it's an obstacle:                      
        if (_rfid_belief_maps.at("ref_map",*iterator) != _free_space_val){
            _rfid_belief_maps.getPosition(*iterator, glob_point);                  
          rel_point = getSubMapRelPoint(glob_point, x_m, y_m, orientation_rad, len_update);
          // remove prob in obstacles
          cout << "1" << endl;
          _tmp_rfid_c_map.atPosition("temp",rel_point) = 0.0;
          cout << "2" << endl;
        }
      }
      cout << "hey4" << endl;
    }

    Eigen::MatrixXf  RadarModel::getProbCondG(std::string layer_i, double x, double sig){

        double W = 0;
        double den = 0;

        // create a copy of the average values
        Eigen::MatrixXf av_mat = _active_area_maps[layer_i];
        
        // cout <<"Layer name: ("<< layer_i <<")" << std::endl;
        // cout <<"x value: ("<< x <<")" << std::endl;
        // cout <<"Layer av. val sumatory: ("<< av_mat.sum() <<")" << std::endl;
        // cout <<"Layer Min Pow: ("<< _active_area_maps[layer_i].minCoeff() <<")" << std::endl;
        // cout <<"Layer Max Pow: ("<< _active_area_maps[layer_i].maxCoeff()  <<")" << std::endl;


        // gaussian prob fddp
        // fddp(x,mu,sigma) = exp( -0.5 * ( (x - mu)/sigma )^2 )   /   (sigma * sqrt(2 pi )) 

        den = sig *sqrt( 2.0 * M_PI) ;
        // cout <<"den: ("<< den <<")" << std::endl;
        
        // reusing matrix for efficiency ...
        av_mat = (x - av_mat.array())/sig;
        // cout <<"t1: ("<< av_mat.sum() <<")" << std::endl;

        av_mat =  av_mat.array().pow(2.0);
        // cout <<"t2: ("<< av_mat.sum() <<")" << std::endl;

        av_mat = -0.5 * av_mat;
        // cout <<"t3: ("<< av_mat.sum() <<")" << std::endl;

        av_mat = av_mat.array().exp()/den;
        // cout <<"t4: ("<< av_mat.sum() <<")" << std::endl;

        // normalize in this space:
        av_mat = av_mat/ (av_mat.sum());
        // cout <<"t5: ("<< av_mat.sum() <<")" << std::endl;

        return av_mat;
    }

    Eigen::MatrixXf  RadarModel::getProbCondLogN(std::string layer_i, double x, double sig){

        double W = 0;
        double den = 0;

        // create a copy of the average values
        Eigen::MatrixXf av_mat = _active_area_maps[layer_i];
        
        //  log-normal prob fddp
        // fddp(x,mu,sigma) = exp( -0.5 * ( (ln(x) - mu)/sigma )^2 )   /   (x * sigma * sqrt(2 pi )) 

        den = x * sig * sqrt( 2.0 * M_PI) ;
        
        // reusing matrix for efficiency ...
        av_mat = x - av_mat.array();
        
        av_mat = ( log(av_mat.array()))/sig;

        av_mat =  av_mat.array().pow(2.0);

        av_mat = -0.5 * av_mat;

        av_mat = av_mat.array().exp()/den;

        // remove nans coming from logarithm
        av_mat = av_mat.unaryExpr([](float v) { return std::isfinite(v)? (float) v : (float) 0.0; });

        // normalize in this space:
        av_mat = av_mat/ (av_mat.sum());

        return av_mat;
    }

    Eigen::MatrixXf  RadarModel::getPhaseProbCond(double ph_i, double f_i){
        std::string layerName = getPhaseLayerName(f_i);
        Eigen::MatrixXf ans = getProbCond(layerName, ph_i, _sigma_phase);
        
        return ans;
    }    

    void RadarModel::PrintRecPower(std::string fileURI,  double f_i){        
        std::string layerName = getPowLayerName(f_i);
        // create a copy of the average values
        Eigen::MatrixXf av_mat = _active_area_maps[layerName];
        PrintProb(fileURI, &av_mat);
    }

    void RadarModel::PrintPhase(std::string fileURI,  double f_i){        
        std::string layerName = getPhaseLayerName(f_i);
        // create a copy of the average values
        Eigen::MatrixXf av_mat = _active_area_maps[layerName];
        PrintProb(fileURI, &av_mat);
    }

    void RadarModel::PrintPowProb(std::string fileURI, double rxPw, double f_i){
        Eigen::MatrixXf prob_mat = getPowProbCond(rxPw, f_i);
        PrintProb(fileURI, &prob_mat);
    }

    void RadarModel::PrintPhaseProb(std::string fileURI, double phi, double f_i){
        Eigen::MatrixXf prob_mat = getPhaseProbCond(phi, f_i);
        PrintProb(fileURI, &prob_mat);
    }


    void RadarModel::PrintBothProb(std::string fileURI, double rxPw, double phi, double f_i){
        Eigen::MatrixXf prob_mat = getPowProbCond(rxPw, f_i).cwiseProduct(getPhaseProbCond(phi, f_i));
        prob_mat = prob_mat/prob_mat.sum();
        PrintProb(fileURI, &prob_mat);
    }

    void RadarModel::PrintProb(std::string fileURI, Eigen::MatrixXf* prob_mat){      
        PrintProb(fileURI, prob_mat, _sizeXaa, _sizeYaa, _resolution);
    }

    void RadarModel::PrintProb(std::string fileURI, Eigen::MatrixXf* prob_mat, double sX, double sY, double res){
        GridMap tempMap;      
        tempMap.setGeometry(Length(sX, sY), res);
        tempMap.add("res", *prob_mat);
        
        float minValue = tempMap["res"].minCoeff();
        float maxValue = tempMap["res"].maxCoeff();

        // cout <<"_________________________________________" << std::endl;
        // cout <<"FILE = '"     << fileURI   << "'" << std::endl;
        // cout <<"MinVal = " << minValue  << "" << std::endl;
        // cout <<"MaxVal = " << maxValue  << "" << std::endl;
        // cout <<"_________________________________________" << std::endl;

        getImage(&tempMap, "res", fileURI);
    }



  
    void RadarModel::debugInfo(){
        std::cout << ".................." << std::endl;
        Length mlen = _active_area_maps.getLength();
        std::cout << "Active area is: "<< mlen(0) << " by " << mlen(1) << " m. (x,y)" <<std::endl;
        std::cout << "Resolution is: " << _resolution<< " m. /cell" << std::endl;
        std::cout << "Active grid is: "<< _active_area_maps.getSize()(0) << " by " << _active_area_maps.getSize()(1) << " cells (i,j axis)" <<std::endl;
        std::cout << ". " << std::endl;
        std::cout << "Total grid Map is: "<< _Ncol << " by " << _Nrow << " cells (i,j axis)" <<std::endl;

        // for (const auto& layer : _active_area_maps.getLayers()) {
        //     std::cout << ((std::string) layer) << std::endl;
        // }
        std::cout << ".................." << std::endl;
    }



    void RadarModel::getImage(std::string layerName, std::string fileURI){
        getImage(&_active_area_maps, layerName, fileURI);
    }

    void RadarModel::saveProbMapDebug(std::string savePATH, int tag_num, int step, double robot_x, double robot_y, double robot_head){
      char buffer [10];
      int n;
      
      std::string fileURI = savePATH + "T";
     
      n=sprintf (buffer, "%01d", tag_num);
      fileURI += std::string(buffer) +"_S";
      n=sprintf (buffer, "%03d", step);
      fileURI += std::string(buffer)+ "_tempMap.png";
      
      std::string layerName = getTagLayerName(tag_num);
      // Some color  ...
      cv::Scalar green( 0, 255, 0 );
      
      // Convert to image.
      cv::Mat image = rfidBeliefToCVImg(layerName);
      
      cv::Point tag_center;

      /// overlay tag position .................................................................................................
      double tx = _tags_coords[tag_num].first;
      double ty =  _tags_coords[tag_num].second;
      tag_center = getPoint( tx, ty );
      cv::circle(image, tag_center , 5, green, 1);

      /// overlay robot position .................................................................................................
      overlayRobotPoseT(robot_x, robot_y, robot_head, image);

      /// overlay active map edges   .............................................................................................
      overlayActiveMapEdges(robot_x, robot_y, robot_head, image);

      /// overlay active map edges   .............................................................................................
      overlayMapEdges( image);

      // clear out obstacles
      //clearObstacles(image);

      // and save
      cv::imwrite( fileURI, image );



    }

    cv::Mat RadarModel::rfidBeliefToCVImg(std::string layer_i){
      cv::Mat image;
      const float minValue = _rfid_belief_maps[layer_i].minCoeff();
      const float maxValue = _rfid_belief_maps[layer_i].maxCoeff();
      // float sum = _rfid_belief_maps[layerName].sum();
      // _rfid_belief_maps[layerName] = _rfid_belief_maps[layerName] / sum;
      // minValue = _rfid_belief_maps[layerName].minCoeff();
      // maxValue = _rfid_belief_maps[layerName].maxCoeff();
      // cout << "max: " << maxValue << endl;
      // cout << "min: " << minValue << endl;
      // cout << "sum: " << sum << endl;
      // cout << "---" << endl;
      // mfc: maxvalue changes, so does what is encoded as black/white and making comparison hard to compare between images ....
      GridMapCvConverter::toImage<unsigned char, 3>(_rfid_belief_maps, layer_i, CV_8UC3, minValue, maxValue, image);
      
      // In order to store a image with ric-coords, we need to flip
      cv::flip(image, image, -1);
      return image;

    }

    void RadarModel::clearObstacles(cv::Mat& image){
      //MFC this method does not work .... dont use it!
      int b = 0;
      int a = 1/b;
      //////////////////////////////////
      
      cv::Vec<unsigned char, 3> red ={ 0, 0, 255 };

      // Initialize result image.
      cv::Mat result = image.clone().setTo(cv::Scalar(255, 255, 255));

      // Convert to image.
      cv::Mat ref_img = rfidBeliefToCVImg("ref_map");

      uint8_t ref_val ;
      // Copy pixels from background to result image, where pixel in mask is 0.
      for (int x = 0; x < image.size().width; x++){
          for (int y = 0; y < image.size().height; y++){
            ref_val =ref_img.at<uint8_t>(y, x) ;
              if (ref_val == 0){
                result.at<cv::Vec3b>(y, x) = red;                  
              } else {
                result.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(y, x);
              }
              if ((ref_val != 0) && (ref_val != 255)){
                std::cout<<"Map image NOT BINARY AT (" << x << ", " <<  y<<") = " << ref_val << std::endl;       
              }
          }
      }

      // there you go ...
      image = result;

      // std::cout<<"Map image (" << ref_img.size().width << ", " <<ref_img.size().height<<") pixels" <<std::endl;
      // std::cout<<"Out image (" << image.size().width << ", " <<image.size().height<<") pixels" <<std::endl;
      // and save
      cv::imwrite( "/tmp/refMap.png", ref_img );


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


    void RadarModel::overlayRobotPoseT(double robot_x, double robot_y, double robot_head, cv::Mat& image){
      grid_map::Index index;    
      cv::Point center;              
      grid_map::Position p;
      cv::Point pentag_points[1][5];

      cv::Scalar red( 0, 0, 255 );
      center = getPoint( robot_x, robot_y );  

      // create a pentagone pointing x+

      int h=4; //pixels?      

      pentag_points[0][0] = cv::Point( center.x  - h,  center.y -   h );
      pentag_points[0][1] = cv::Point( center.x  - h,  center.y +   h );
      pentag_points[0][2] = cv::Point( center.x     ,  center.y + 2*h );
      pentag_points[0][3] = cv::Point( center.x  + h,  center.y +   h );
      pentag_points[0][4] = cv::Point( center.x  + h,  center.y -   h );
      const cv::Point* pts[1] = { pentag_points[0] };
      rotatePoints(pentag_points[0] ,5,center.x, center.y, robot_head );
      int npts[] = { 5 };
      cv::fillPoly( image,pts,npts,1,red,8 );

    }

    void RadarModel::rotatePoints( cv::Point* points, int npts, int cxi, int cyi, double ang){
      double offsetx, offsety, cx, cy, px, py, cosA, sinA;

      cx = (double) cxi;
      cy = (double) cyi;

      cosA = cos(ang);
      sinA = sin(ang);

      for(int i=0; i<npts; i++){
          px = points[i].x;
          py = points[i].y;

          offsetx = (cosA * (px - cx)) + (sinA * (py - cy));
          offsety = -(sinA * (px - cx)) + (cosA * (py - cy));

          points[i].x =( (int) offsetx) + cxi;
          points[i].y =( (int) offsety) + cyi;
      }

    }


    void RadarModel::overlayRobotPose(double robot_x, double robot_y, double robot_head, cv::Mat& image){
      
      
      cv::Point center;                    
      
      cv::Scalar red( 0, 0, 255 );
      center = getPoint( robot_x, robot_y );
      cv::circle(image, center , 5, red, -1);

    }

    void RadarModel::PrintRefMapWithTags(std::string fileURI){
      std::vector<std::pair<cv::Scalar,std::string>> color_list;

      // Some color  ...
      cv::Scalar red( 0, 0, 255 );
      color_list.push_back(std::make_pair(red,"red"));
      cv::Scalar green( 0, 255, 0 );
      color_list.push_back(std::make_pair(green,"green"));
      cv::Scalar blue( 255, 0, 0 );
      color_list.push_back(std::make_pair(blue,"blue"));      
      cv::Scalar yellow( 0, 255, 255 );  
      color_list.push_back(std::make_pair(yellow,"yellow"));
      cv::Scalar blueviolet(226,138,43 );  
      color_list.push_back(std::make_pair(blueviolet,"blueviolet"));
      cv::Scalar turquoise(224,208,64);
      color_list.push_back(std::make_pair(turquoise,"turquoise"));
      cv::Scalar orange(165,0,255);
      color_list.push_back(std::make_pair(orange,"orange"));
      cv::Scalar pink(192,203,255);
      color_list.push_back(std::make_pair(pink,"pink"));
      cv::Scalar chocolate(105,30,210);
      color_list.push_back(std::make_pair(chocolate,"chocolate"));
      cv::Scalar dodgerblue(144,255,30);
      color_list.push_back(std::make_pair(dodgerblue,"dodgerblue"));

      // Convert to image.
      cv::Mat image = rfidBeliefToCVImg("ref_map");

      cv::Point center;

      for (int i = 0; i < _tags_coords.size(); i++){
            double x = _tags_coords[i].first;
            double y =  _tags_coords[i].second;
            center = getPoint( x, y );
            cv::circle(image, center , 5, color_list[i%color_list.size()].first, -1);

      }
      /// overlay active map edges   .............................................................................................
      overlayMapEdges( image);

      cv::imwrite( fileURI, image );



    }


    void RadarModel::getImage(GridMap* gm,std::string layerName, std::string fileURI){

      // Convert to image.
      cv::Mat image;
      const float minValue = (*gm)[layerName].minCoeff();
      const float maxValue = (*gm)[layerName].maxCoeff();

      GridMapCvConverter::toImage<unsigned char, 3>(*gm, layerName, CV_8UC3, minValue, maxValue, image);  

      // In order to store a image with ric-coords, we need to flip
      cv::flip(image, image, -1);
      
      cv::imwrite( fileURI, image );

    }

    double RadarModel::getTotalWeight(int tag_i){
      return getTotalWeight(( _Ncol - 1 ) * (_resolution / 2.0), ( _Nrow - 1 ) * (_resolution / 2.0), 0, _Ncol*_resolution, _Nrow*_resolution, tag_i);
    }

    double RadarModel::getTotalWeight(double x, double y, double orientation, int tag_i){
      return getTotalWeight(x, y, orientation, _sizeXaa, _sizeYaa, tag_i);
    }

    double RadarModel::getTotalWeight(double x, double y, double orientation, double size_x, double size_y, int tag_i){
      // TODO: I'm not using the orientation. Maybe it would be better to use a polygon iterator, 
      //     so we can rotate edges around the center and have a more flexible thing

      //submapStartIndex the start index of the submap, typically top-left index.
      grid_map::Index submapStartIndex, submapEndIndex,submapBufferSize;
      grid_map::Position submapStartPosition(x+(size_x/2), y+(size_y/2));
      grid_map::Position submapEndPosition(x-(size_x/2), y-(size_y/2));
    
      if (!_rfid_belief_maps.getIndex(submapStartPosition,submapStartIndex)){  
        submapStartIndex = grid_map::Index(0,0);
        //std::cout<<"Clip start!" << std::endl;
      }

      if (!_rfid_belief_maps.getIndex(submapEndPosition,submapEndIndex)){  
        Size siz = _rfid_belief_maps.getSize();
        submapEndIndex = grid_map::Index(siz(0)-1,siz(1)-1);
        //std::cout<<"Clip end!" << std::endl;
      }

      submapBufferSize = submapEndIndex - submapStartIndex;    

      grid_map::SubmapIterator iterator(_rfid_belief_maps, submapStartIndex, submapBufferSize);


      // std::cout<<"\nGet prob.:" << std::endl;
      // std::cout<<" Centered at Position (" << x << ", " << y << ") m. / Size (" << size_x << ", " << size_y << ")" << std::endl;
      // std::cout<<" Start pose (" << submapStartPosition(0) << ", " << submapStartPosition(1) << ") m. to pose " << submapEndPosition(0) << ", " << submapEndPosition(1) << ") m."<< std::endl;
      // std::cout<<" Start Cell ("  << submapStartIndex(0) << ", " << submapStartIndex(1) << ") to cell("  << submapEndIndex(0) << ", " << submapEndIndex(1) << ")"<< std::endl;

      return getTotalWeight(x, y, orientation, iterator, tag_i);
    }


    double RadarModel::getTotalWeight(double x, double y, double orientation, grid_map::SubmapIterator iterator, int tag_i){

      double total_weight;
      Position point;

      std::string tagLayerName = getTagLayerName(tag_i);

      total_weight = 0;
      for (iterator; !iterator.isPastEnd(); ++iterator) {                  
        _rfid_belief_maps.getPosition(*iterator, point);                  
        // check if is inside global map
        if (_rfid_belief_maps.isInside(point)){
          // We don't add belief from positions considered obstacles...                      
          if (_rfid_belief_maps.atPosition("ref_map",point)==_free_space_val){
            total_weight += _rfid_belief_maps.atPosition(tagLayerName,point);
          }
        }
      }
      return total_weight;

    }

    void RadarModel::getImageDebug(std::string layerName, std::string fileURI){

      // Convert to image.
      cv::Mat image;
      const float minValue = _active_area_maps[layerName].minCoeff();
      const float maxValue = _active_area_maps[layerName].maxCoeff();

      GridMapCvConverter::toImage<unsigned char, 3>(_active_area_maps, layerName, CV_8UC3, minValue, maxValue, image);
      
      cv::Scalar green( 0, 255, 0 );
      cv::Scalar blue( 255, 0, 0 );      
      cv::Scalar red( 0, 0, 255 );
      cv::Scalar yellow( 0, 255, 255 );
      
      grid_map::Index index;
      

      double maxX=  _active_area_maps.getLength().x()/2;
      double maxY=  _active_area_maps.getLength().y()/2;

      grid_map::Position p(maxX,  maxY );
      _active_area_maps.getIndex(p,index);      
      cv::Point gree( index.x(), index.x() );
      std::cout<<"Green: (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

      p = Position(maxX,  -maxY);
      _active_area_maps.getIndex(p,index);      
      cv::Point blu( index.y(), index.x() );
      std::cout<< "Blue (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

      p = Position(-maxX,  -maxY);
      _active_area_maps.getIndex(p,index);  
      cv::Point re( index.y(), index.x() );
      std::cout<<"Red (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

      p = Position(-maxX,  maxY);
      _active_area_maps.getIndex(p,index);  
      cv::Point yell( index.y(), index.x() );
      std::cout<<"Yellow (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;
      
      cv::circle(image, gree , 20, green, -1);
      cv::circle(image, blu , 20, blue, -1);
      cv::circle(image, re ,  20, red, -1);
      cv::circle(image, yell , 20, yellow, -1);
      
      cv::Point triang_points[1][3];
      double h=0.2;
      
      p = Position(h/2.0,  0);
      _active_area_maps.getIndex(p,index);        
      triang_points[0][0] = cv::Point( index.y(), index.x() );
      std::cout<<"p (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

      p = Position(-h/2.0,  -h/2.0);
      _active_area_maps.getIndex(p,index);  
      triang_points[0][1] = cv::Point( index.y(), index.x() );
      std::cout<<"p (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

      p = Position(-h/2.0,  h/2.0);
      _active_area_maps.getIndex(p,index);  
      triang_points[0][2] = cv::Point( index.y(), index.x() );
      std::cout<<"p (" << p(0) << ", " << p(1)<<") m. == Cell("  << index(0) << ", " << index(1) << ")" <<std::endl;

      const cv::Point* ppt[1] = { triang_points[0] };
      int npt[] = { 3 };
      cv::fillPoly( image,ppt,npt,1,red,8 );

      
      // Rotate 90 Degrees Clockwise To get our images to have increasing X to right and increasing Y up
      cv::transpose(image, image);
      cv::flip(image, image, 1);
      cv::imwrite( fileURI, image );

    }

    /*
      cv::Point rook_points[1][3];

      const cv::Point* ppt[1] = { rook_points[0] };
      int npt[] = { 3 };
      //cv::fillPoly( image,ppt,npt,1,yellow,8 );
      // grid_map::Polygon polygon;
      // polygon.setFrameId(_active_area_maps.getFrameId());
      // polygon.addVertex(Position( 0, h/2 ));
      // polygon.addVertex(Position(  -h/2,  h/2 ));
      // polygon.addVertex(Position( h/2, -h/2 ));
      // for (grid_map::PolygonIterator iterator(map_, polygon); !iterator.isPastEnd(); ++iterator) {
      //     map_.at("type", *iterator) = 1.0;
      // }

    */

    std::string RadarModel::getPowLayerName(double freq_i){
      return "P_"+getLayerName(freq_i/1e6);
    }

    std::string RadarModel::getPhaseLayerName(double freq_i){
      return "D_"+getLayerName(freq_i/1e6);
    }

    std::string RadarModel::getTagLayerName(int tag_num){
      return std::to_string(tag_num);
    }
    

    std::string RadarModel::getLayerName(double x){

        // Create an output string stream
        std::ostringstream streamObj3;

        // Set Fixed -Point Notation
        streamObj3 << std::fixed;

        // Set precision to 2 digits
        streamObj3 << std::setprecision(2);

        //Add double to stream
        streamObj3 << x;

        // Get string from output string stream
        std::string strObj3 = streamObj3.str();

        return strObj3;
    }



    /////////////////////////////////////////////////




std::vector<double> RadarModel::range(double start, double stop, double step)
{
 std::vector<double> ans;
 double currVal = start;
   while (currVal <= stop){
     ans.push_back(currVal);
     currVal += step;
   }
 return ans;
}

double RadarModel::antennaPlaneLoss(double angleRad ){
  double g_ans;
  double g_i = -22.6;
  double g_p = -22.6;
  int i;
  int p;
  double ang_i = 0;
  double ang_p = 0;
  double m = 0;
  double g_o = 0;
  double angleDeg = angleRad*180.0/M_PI;

  if (fabs(angleDeg)>180.0){
      std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
      std::cout<<"Angle (" << angleDeg<<") deg. " <<std::endl;
      std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
  }

  // gain list entries start at -180 degrees to 180 in steps of 15.
  double  fIndex = (angleDeg + 165.0) / 15.0;

  // we dont want to index out of the range ...
  i = std::max( std::min( (int) ceil(fIndex), 24), 0);
  p = std::max( std::min( (int) floor(fIndex), 24), 0);


  ang_i = (i * 15.0) - 180.0;
  ang_p = (p * 15.0) - 180.0;

  g_i = ANTENNA_LOSSES_LIST[i];
  g_p = ANTENNA_LOSSES_LIST[p];

  if (i!=p){
    m = (g_i-g_p)  / (ang_i-ang_p);
  }

  g_o = g_i - (m * ang_i);
  g_ans = m * angleDeg + g_o;

  //

  return g_ans;
}

float RadarModel::sign(float x){
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return 0.0;
}

void RadarModel::getSphericCoords(double x, double y, double& r, double& phi){
  r = sqrt(x*x+y*y);
  phi = atan2(y,x);

}

double RadarModel::received_power_friis(double tag_x, double tag_y, double freq, double txtPower) {
     double rxPower = txtPower;
     // build spline to interpolate antenna gains;
     std::vector<double> xVec(ANTENNA_ANGLES_LIST, ANTENNA_ANGLES_LIST + 25);
     Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xVec.data(), xVec.size());
     std::vector<double> yVec(ANTENNA_LOSSES_LIST, ANTENNA_LOSSES_LIST + 25);
     Eigen::VectorXd yvals= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yVec.data(), yVec.size());
     SplineFunction antennaGainsModel(xvals, yvals);

     rxPower = received_power_friis(tag_x, tag_y, freq, txtPower, antennaGainsModel);
     
     return rxPower;
 }


double RadarModel::received_power_friis(double tag_x, double tag_y, double freq, double txtPower, SplineFunction antennaGainsModel) {
     double rxPower = txtPower;
     double phi;
     double r;
     // todo: use a threshold instead of exact value
     if (!((tag_x==0) && (tag_y==0))){
         getSphericCoords(tag_x,tag_y, r, phi);

         rxPower = received_power_friis_polar(r, phi, freq, txtPower, antennaGainsModel);
     }
     
     return rxPower;
 }

double RadarModel::received_power_friis_polar(double tag_r, double tag_h, double freq, double txtPower, SplineFunction antennaGainsModel) {
     double rxPower = txtPower;
     double ant1, antL, propL;
     double lambda =  C/freq;
    // otherwise friis approach does not apply
     if (tag_r>lambda){
         /*
          SIMPLIFICATION!!! TAG is OMNIDIRECTIONAL
          (a.k.a. don't have tag radiation pattern and
          Here they say it's ok https://www.hindawi.com/journals/ijap/2013/194145/tab4/
         */
         ant1 = antennaGainsModel.interpRad(tag_h);

         antL =  TAG_LOSSES + ant1;

         // propagation losses
         propL = LOSS_CONSTANT - (20 * log10  (tag_r * freq)) ;
         // signal goes from antenna to tag and comes back again, so we double the losses
         rxPower +=  2*antL + 2*propL ;
     }

     if (rxPower > txtPower){
       std::cout << endl << endl << "ERROR! MORE POWER RECEIVED THAN TRANSMITTED!!!!!!!!!!!!! " << endl;
       std::cout << "Relative tag polar pose: (" << tag_r << " m.," << tag_h*180/M_PI << " deg.)" << endl;
       std::cout << "At Freq.: (" << freq/1e6 << " MHz.)" << endl;
       std::cout << "Lambda: (" << lambda << " m.)" << endl;
       std::cout << "Tx Pw.: (" << txtPower << " dB)" << endl;
       std::cout << "Rx Pw.: (" << rxPower << " dB)" << endl;
       std::cout << "Antenna losses: (" << antL << " dB)" << endl;
       std::cout << "Propagation losses: (" << propL << " dB)" << endl;
       std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl << endl << endl ;
     }
     
     return rxPower;
 }

double RadarModel::phaseDifference(double tag_x, double tag_y, double freq) {
  double phi;
  double r;

  getSphericCoords(tag_x,tag_y, r, phi);
  double phase = PHASE_CONSTANT * freq * r;
  phase = fmod(phase, M_PI);
  return phase;

}


void RadarModel::activeAreaFriis(double freq, double txtPower, double sensitivity, double distStep, double& minX, double& minY, double& maxX, double& maxY) {

  double currX = 0.0;
  double currY = 0.0;
  double currRxPower = txtPower;

  std::cout << "txtPower: " << txtPower << "\n";
  std::cout << "sensitivity: " << sensitivity << "\n";

  // build spline to interpolate antenna gains;
  std::vector<double> xVec(ANTENNA_ANGLES_LIST, ANTENNA_ANGLES_LIST + 25);
  Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xVec.data(), xVec.size());
  std::vector<double> yVec(ANTENNA_LOSSES_LIST, ANTENNA_LOSSES_LIST + 25);
  Eigen::VectorXd yvals= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yVec.data(), yVec.size());
  SplineFunction antennaGainsModel(xvals, yvals);

  // get max X distance within sensitivity
  do{
    currX = currX+distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower, antennaGainsModel);
  } while(currRxPower>sensitivity);
  maxX =currX-distStep; // because last iteration should be under sensitivity

  // get min X distance within sensitivity
  currX = 0.0;
  currY = 0.0;
  currRxPower = txtPower;

  do{
    currX = currX-distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower, antennaGainsModel);
  } while(currRxPower>sensitivity);
  minX =currX+distStep; // because last iteration should be under sensitivity

  // get max Y distance within sensitivity
  currX = 0.0;
  currY = 0.0;
  currRxPower = txtPower;

  do{
    currY = currY+distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower, antennaGainsModel);
  } while(currRxPower>sensitivity);
  maxY =currY-distStep; // because last iteration should be under sensitivity

  // get min Y distance within sensitivity
  currX = 0.0;
  currY = 0.0;
  currRxPower = txtPower;

  do{
    currY = currY-distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower, antennaGainsModel);
  } while(currRxPower>sensitivity);
  minY =currY+distStep; // because last iteration should be under sensitivity

}

std::pair<int, std::pair<int, int>> RadarModel::findTagFromBeliefMap(int num_tag){
  
  // Access the belief map of every tag
  std::string layerName = getTagLayerName(num_tag);
  GridMap::Matrix& grid = _rfid_belief_maps[layerName];
  
  std::pair<int,int> tag(0,0);
  double powerRead = 0;
  // for(int row=0; row < grid.getLength().x(); row++)
  // {
  //   for(int col=0; col < grid.getLength().y(); col++)
  //   {

  for(GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd(); ++iterator)
  { 
      const Index index(*iterator);
      // For every cell, analyse the surrounding area
      double tmp_power = 0.0;
      tmp_power = grid(index(0), index(1));
      int buffer_size = 3;
      if (index(0) > buffer_size and index(0) <= _rfid_belief_maps.getLength().x() - buffer_size){
        if(index(1) > buffer_size and index(1) <= _rfid_belief_maps.getLength().y() - buffer_size){
          // std::cout << "I: " << index(0) << "," << index(1) << endl;
          Index submapStartIndex(index(0) - buffer_size, index(1) - buffer_size);
          Index submapBufferSize(buffer_size, buffer_size);
          for (grid_map::SubmapIterator sub_iterator(_rfid_belief_maps, submapStartIndex, submapBufferSize); 
            !sub_iterator.isPastEnd(); ++sub_iterator) {
            Index sub_index(*sub_iterator);
            // std::cout << "I: " << sub_index(0) << "," << sub_index(1) << endl;
            tmp_power += grid(sub_index(0), sub_index(1));
          }
        }  
      }
      
      
      // for (int i = -3; i <= 3; i++){
      //   for (int j = -3; j <= 3; j++){
      //     tmp_power = tmp_power + grid.getCell(row, col); 
      //   }
      // }
      // tmp_power = grid.getCell(row, col); 
      if(tmp_power > powerRead)
      {
        powerRead = tmp_power;
        
        // Normalise the tag coordinate to follow Ricc's system
        tag.first  = _rfid_belief_maps.getLength().x() - index(0);
        tag.second = _rfid_belief_maps.getLength().y() - index(1);
      }

    // }
  }
  std::pair<int, std::pair<int, int>> final_return (powerRead, tag);

  // cout << "Value read: " << powerRead << endl;
  return final_return;
}

void RadarModel::normalizeRFIDLayer(std::string layerName){
  double totalW = _rfid_belief_maps[layerName].sum();
  if (totalW > 0){
    _rfid_belief_maps[layerName] = _rfid_belief_maps[layerName]/totalW;
  }

  // double minW = _rfid_belief_maps[layerName].minCoeff();
  // double maxW = _rfid_belief_maps[layerName].maxCoeff();
  // _rfid_belief_maps.add("min", minW);
  // _rfid_belief_maps.add("max", minW);
  // _rfid_belief_maps[layerName] = (_rfid_belief_maps[layerName] - _rfid_belief_maps["min"])/ (maxW - minW);
}

void RadarModel::normalizeRFIDMap(){
  std::string layerName;
  // For every tag map, calculate the sum over the pixels and
  // divide each pixel intensity by the sum
  for (int i=0; i < _numTags; ++i){
    layerName = getTagLayerName(i);
    double totalW = _rfid_belief_maps[layerName].sum();
    if (totalW > 0){
      _rfid_belief_maps[layerName] = _rfid_belief_maps[layerName]/totalW;
    }
  }
}


void RadarModel::clearObstacleCellsRFIDMap(){
  Position point;
  std::string layerName;

  for (int i=0; i < _numTags; ++i){
    layerName = getTagLayerName(i);
    for (grid_map::GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd(); ++iterator) {                  
      // get cell center of the cell in the map frame.            
      _rfid_belief_maps.getPosition(*iterator, point);
      if(_rfid_belief_maps.atPosition("ref_map",point) != _free_space_val){
        _rfid_belief_maps.at(layerName,*iterator) = 0.0;
        // count++;
      }               
    }
  }
}

double RadarModel::getNormalizingFactorBayesRFIDActiveArea(double x_m, double y_m, double orientation_rad, string tagLayerName){
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;
  double prior, likelihood;
  double bayes_den = 0.0;

  // update_edges = getSubMapEdges(x_m, y_m, orientation_rad, 5);
  update_edges = getActiveMapEdges(x_m, y_m, orientation_rad);
  for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges); !iterator.isPastEnd(); ++iterator){
      // get point value in reference map to check if it's an obstacle:                      
      if (_rfid_belief_maps.at("ref_map",*iterator ) == _free_space_val){
        _rfid_belief_maps.getPosition(*iterator, glob_point);                  
        rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);                  
        // likelihood = _tmp_rfid_c_map.atPosition("temp",rel_point);
        likelihood = _active_area_maps.atPosition("temp",rel_point);
        prior = _rfid_belief_maps.at(tagLayerName,*iterator);
        bayes_den += likelihood * prior;
      }
  }
  return bayes_den;
}

double RadarModel::getNormalizingFactorBayesFullMap(double x_m, double y_m, double orientation_rad, string tagLayerName){
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;
  double prior, likelihood;
  double bayes_den = 0.0;

  for (grid_map::GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd(); ++iterator) {  
      // get point value in reference map to check if it's an obstacle:                      
      if (_rfid_belief_maps.at("ref_map",*iterator ) == _free_space_val){
        _rfid_belief_maps.getPosition(*iterator, glob_point);
        rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);                  
        likelihood = _rfid_belief_maps.atPosition("temp",glob_point);
        prior = _rfid_belief_maps.at(tagLayerName,*iterator);
        bayes_den += likelihood * prior;
      }
  }
  return bayes_den;
}

Eigen::MatrixXf RadarModel::getNegProb(std::string layer_i, double x, double sigm){
 
 Eigen::MatrixXf friis_mat = _active_area_maps[layer_i];
 
 // gaussian cdp
 // cdp(x,mu,sigma) = 0.5 * ( 1 + erf( (x - mu) / (sigma * sqrt(2)) ) 
 
 Eigen::MatrixXf erf_mat = (x - friis_mat.array()) / (sigm * sqrt(2.0));
 
 erf_mat = 0.5 + 0.5*erf_mat.array().erf();
 return erf_mat;
 }

 Eigen::MatrixXf RadarModel::getNegProbRFIDCriterion( double x, double sigm){
    Eigen::MatrixXf friis_mat = _tmp_rfid_c_map["temp"];
    // gaussian cdp
    // cdp(x,mu,sigma) = 0.5 * ( 1 + erf( (x - mu) / (sigma * sqrt(2)) ) 
    Eigen::MatrixXf erf_mat = (x - friis_mat.array()) / (sigm * sqrt(2.0));
    erf_mat = 0.5 + 0.5*erf_mat.array().erf();
    return erf_mat;
 }

 Eigen::MatrixXf RadarModel::getIntervProb(std::string layer_i, double x, double sigm){
      //Size siz = _active_area_maps.getSize();

      // probability of receiving less than X1 in every cell
      Eigen::MatrixXf x1_mat = getNegProb( layer_i+"_low", x, sigm);
      //std::cout << "x1_mat: " << x1_mat.sum()/(siz(0)*siz(1)) << endl;

      // probability of receiving less than X2 in every cell
      Eigen::MatrixXf x2_mat = getNegProb( layer_i+"_hig", x, sigm);
      //std::cout << "x2_mat: " << x1_mat.sum()/(siz(0)*siz(1)) << endl;

      // probability of receiving between X1 and X2 in every cell
      Eigen::MatrixXf interv_mat = (x2_mat - x1_mat).array().abs();
      //std::cout << "Diff: " << interv_mat.sum()/(siz(0)*siz(1)) << endl<< endl;

 
 return interv_mat;
 }


cv::Point RadarModel::getPoint(double x_m, double y_m){
    Length mlen = _rfid_belief_maps.getLength();  
    grid_map::Position orig = _rfid_belief_maps.getPosition();  
    double min_x = orig(0) - mlen(0)/2 + _resolution; 
    double max_x = orig(0) + mlen(0)/2 - _resolution; 
    double min_y = orig(1) - mlen(1)/2 + _resolution; 
    double max_y = orig(1) + mlen(1)/2 - _resolution; 


    grid_map::Index index;
      
    grid_map::Position p(x_m,  y_m);                    
                      
    if (!_rfid_belief_maps.getIndex(p,index)) {
         //std::cout<<" Position ("  << p(0) << ", " << p(1) << ") is out of _rfid_belief_maps bounds!!" <<std::endl;  
         x_m = std::min( std::max(x_m, min_x) , max_x);
         y_m = std::min( std::max(y_m, min_y) , max_y);
         p = grid_map::Position (x_m,  y_m);
         _rfid_belief_maps.getIndex(p,index);
    }

      // cast from gridmap indexes to opencv indexes 
      int cv_y = (_Nrow-1) - index.x();
      int cv_x = (_Ncol-1) - index.y();
      //std::cout<<"Which equals to opencv cell ("  << cv_x << ", " << cv_y << ") " << std::endl;

    return cv::Point( cv_x, cv_y );
}

grid_map::Position RadarModel::fromPoint(cv::Point cvp) {
  grid_map::Position p;
  int gm_x, gm_y;
  
  // cast from opencv indexes to  gridmap indexes
  gm_x = (_Nrow-1) - cvp.y;
  gm_y = (_Ncol-1) - cvp.x;

  grid_map::Index index(gm_x,gm_y);

    if (!_rfid_belief_maps.getPosition(index,p)) {
         //std::cout<<" Index ("  << index(0) << ", " << index(1) << ") is out of _rfid_belief_maps bounds!!" <<std::endl;  
         gm_x = std::min( std::max(gm_x, 0) , _Nrow-1);
         gm_y = std::min( std::max(gm_y, 0) , _Ncol-1);
         index = grid_map::Index(gm_x,gm_y);
         _rfid_belief_maps.getPosition(index,p);
    }
  
  return p;
}

     
void RadarModel::overlayMapEdges( cv::Mat image){      
      cv::Point square_points[1][4];
      cv::Scalar red( 0, 255, 255 );
      int cv_y, cv_x;

      cv_y = (_Nrow-1);
      cv_x = (_Ncol-1);


      square_points[0][0] = cv::Point( 0, 0 );
      square_points[0][1] = cv::Point( 0, cv_y );
      square_points[0][2] = cv::Point( cv_x, cv_y );
      square_points[0][3] = cv::Point( cv_x, 0 );

      const cv::Point* pts[1] = { square_points[0] };
      int npts[] = { 4 };
      cv::polylines(image,pts,npts,1, true,red);
     
}
        
void RadarModel::overlayActiveMapEdges(double robot_x, double robot_y, double robot_head, cv::Mat image){      
      cv::Point square_points[1][4];
      cv::Point cvrobot;
      cv::Scalar red( 0, 0, 255 );

      Length mlen = _active_area_maps.getLength();
      cvrobot = getPoint(robot_x, robot_y);

      square_points[0][0] = getPoint(robot_x - mlen(0)/2, robot_y - mlen(1)/2);
      square_points[0][1] = getPoint(robot_x - mlen(0)/2, robot_y + mlen(1)/2);
      square_points[0][2] = getPoint(robot_x + mlen(0)/2, robot_y + mlen(1)/2);
      square_points[0][3] = getPoint(robot_x + mlen(0)/2, robot_y - mlen(1)/2);
      const cv::Point* pts[1] = { square_points[0] };
      rotatePoints(square_points[0] ,4, cvrobot.x, cvrobot.y,robot_head );
      int npts[] = { 4 };
      cv::polylines(image,pts,npts,1, true,red);
     
}




grid_map::Polygon RadarModel::getActiveMapEdges(double robot_x, double robot_y, double robot_head){      
      Length mlen = _active_area_maps.getLength();
      grid_map::Polygon polygon;
            
      // trying not to replicate code, I'm using the cv points one that works
      cv::Point cvrobot = getPoint(robot_x, robot_y);
      cv::Point pts[4];
      pts[0] = getPoint(robot_x - mlen(0)/2, robot_y - mlen(1)/2);
      pts[1] = getPoint(robot_x - mlen(0)/2, robot_y + mlen(1)/2);
      pts[2] = getPoint(robot_x + mlen(0)/2, robot_y + mlen(1)/2);
      pts[3] = getPoint(robot_x + mlen(0)/2, robot_y - mlen(1)/2);
      
      rotatePoints(pts ,4, cvrobot.x, cvrobot.y,robot_head );

      // cast to gridmap points and add to polygon
      polygon.setFrameId(_rfid_belief_maps.getFrameId());
      for(int i=0; i<4; i++){
          polygon.addVertex(fromPoint(pts[i]));
      }

      return polygon;
}

grid_map::Polygon RadarModel::getSubMapEdges(double robot_x, double robot_y, double robot_head, double len){      
      grid_map::Polygon polygon;
            
      // trying not to replicate code, I'm using the cv points one that works
      cv::Point cvrobot = getPoint(robot_x, robot_y);
      cv::Point pts[4];
      pts[0] = getPoint(robot_x - len/2, robot_y - len/2);
      pts[1] = getPoint(robot_x - len/2, robot_y + len/2);
      pts[2] = getPoint(robot_x + len/2, robot_y + len/2);
      pts[3] = getPoint(robot_x + len/2, robot_y - len/2);
      
      rotatePoints(pts ,4, cvrobot.x, cvrobot.y,robot_head );

      // cast to gridmap points and add to polygon
      polygon.setFrameId(_rfid_belief_maps.getFrameId());
      for(int i=0; i<4; i++){
          polygon.addVertex(fromPoint(pts[i]));
      }

      return polygon;
}


  Position RadarModel::getRelPoint(Position glob_point, double x_m, double  y_m, double orientation_rad){
      double rel_x, rel_y;
      Position rel_point;
      // we may need these for castings ...
      Length mlen = _active_area_maps.getLength();  
      grid_map::Position orig = _active_area_maps.getPosition();  
      double min_x = orig(0) - mlen(0)/2 + _resolution; 
      double max_x = orig(0) + mlen(0)/2 - _resolution; 
      double min_y = orig(1) - mlen(1)/2 + _resolution; 
      double max_y = orig(1) + mlen(1)/2 - _resolution; 

          rel_x  =  (glob_point.x() - x_m) * cos(orientation_rad) + (glob_point.y() - y_m) * sin(orientation_rad);
          rel_y  = -(glob_point.x() - x_m) * sin(orientation_rad) + (glob_point.y() - y_m) * cos(orientation_rad);
          rel_point = Position(rel_x, rel_y );
                              
        // because of rounding, we may have to adjust this
        if (!_active_area_maps.isInside(rel_point)) {
              rel_x = std::min( std::max(rel_x, min_x) , max_x);
              rel_y = std::min( std::max(rel_y, min_y) , max_y);
              rel_point = Position(rel_x, rel_y );
        }
        return rel_point;
}


Position RadarModel::getSubMapRelPoint(Position glob_point, double x_m, double  y_m, double orientation_rad, double len){
  double rel_x, rel_y;
  Position rel_point;
  // we may need these for castings ...
  grid_map::Position orig = _active_area_maps.getPosition();  
  double min_x = orig(0) - len/2 + _resolution; 
  double max_x = orig(0) + len/2 - _resolution; 
  double min_y = orig(1) - len/2 + _resolution; 
  double max_y = orig(1) + len/2 - _resolution; 

      rel_x  =  (glob_point.x() - x_m) * cos(orientation_rad) + (glob_point.y() - y_m) * sin(orientation_rad);
      rel_y  = -(glob_point.x() - x_m) * sin(orientation_rad) + (glob_point.y() - y_m) * cos(orientation_rad);
      rel_point = Position(rel_x, rel_y );
                          
    // because of rounding, we may have to adjust this
    if (!_active_area_maps.isInside(rel_point)) {
          rel_x = std::min( std::max(rel_x, min_x) , max_x);
          rel_y = std::min( std::max(rel_y, min_y) , max_y);
          rel_point = Position(rel_x, rel_y );
    }
    return rel_point;
}

// use me to plot a white blob instead of a measurement. Used to check transforms
void RadarModel::addMeasurement0(double x_m, double y_m, double orientation_deg, double rxPower, double phase, double freq, int i){
  double rel_x, rel_y, orientation_rad;
  double glob_x, glob_y;
  Position rel_point, glob_point;
  grid_map::Polygon polygon;

  std::string tagLayerName = getTagLayerName(i);
  
  orientation_rad = orientation_deg * M_PI/180.0;
  
  polygon = getActiveMapEdges(x_m, y_m, orientation_rad);


  for (grid_map::PolygonIterator iterator(_rfid_belief_maps, polygon); !iterator.isPastEnd(); ++iterator){
      _rfid_belief_maps.at(tagLayerName,*iterator) = 1;
    }
}

// use me to plot the pdf instead of a bayesian update. Used to check transforms!
void RadarModel::addMeasurement1(double x_m, double y_m, double orientation_deg, double rxPower, double phase, double freq, int i){
  double rel_x, rel_y, orientation_rad;
  double glob_x, glob_y;
  Position rel_point, glob_point;
  grid_map::Polygon polygon;
  Eigen::MatrixXf prob_mat;
  std::string tagLayerName = getTagLayerName(i);
  
  orientation_rad = orientation_deg * M_PI/180.0;
  
  if (rxPower>SENSITIVITY){
    prob_mat = getPowProbCond(rxPower, freq);
    createTempProbLayer(prob_mat, x_m, y_m, orientation_deg);
    
    // normalize bt 0-1
    prob_mat = prob_mat / prob_mat.maxCoeff();

    polygon = getActiveMapEdges(x_m, y_m, orientation_rad);
    for (grid_map::PolygonIterator iterator(_rfid_belief_maps, polygon); !iterator.isPastEnd(); ++iterator){
            if (_rfid_belief_maps.at("ref_map",*iterator) == _free_space_val){
              // get the relative point
              _rfid_belief_maps.getPosition(*iterator, glob_point);                  
              rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);            
              _rfid_belief_maps.at(tagLayerName,*iterator) = _active_area_maps.atPosition("temp",rel_point); 
            }
    }
  }
}

// use me to plot the pdf instead of a bayesian update. Now without walls!
void RadarModel::addMeasurement2(double x_m, double y_m, double orientation_deg, double rxPower, double phase, double freq, int i){
  double rel_x, rel_y, orientation_rad;
  double glob_x, glob_y;
  Position rel_point, glob_point;
  grid_map::Polygon polygon;
  Eigen::MatrixXf prob_mat;
  std::string tagLayerName = getTagLayerName(i);
  
  orientation_rad = orientation_deg * M_PI/180.0;
  
  if (rxPower>SENSITIVITY){
    prob_mat = getPowProbCond(rxPower, freq);
    // normalize bt 0-1
    prob_mat = prob_mat / prob_mat.maxCoeff();
    _active_area_maps.add("temp", prob_mat);     

    polygon = getActiveMapEdges(x_m, y_m, orientation_rad);
    for (grid_map::PolygonIterator iterator(_rfid_belief_maps, polygon); !iterator.isPastEnd(); ++iterator){ 
              // get the relative point
              _rfid_belief_maps.getPosition(*iterator, glob_point);                  
              rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);            
              _rfid_belief_maps.at(tagLayerName,*iterator) = _active_area_maps.atPosition("temp",rel_point);    
    }
  }
}



//So, robot at pr (x,y,orientation) (long, long, int) receives rxPower,phase,freq from tag i . 
//TODO: this is a simplistic model that just "ignores" walls: but they do have absortion and reduce received power at their locations...
// this one ignores active areas ....
void RadarModel::addMeasurement3(double x_m, double y_m, double orientation_deg, double rxPower, double phase, double freq, int i){


  //1. obtain rel dist and friis to all points    ..................................................
  cout << x_m << " " << y_m << " " << orientation_deg << " " << rxPower << endl;
  Eigen::MatrixXf rxPw_mat, likl_mat;
  double tag_x, tag_y, tag_r, tag_h, rxP;
  double pdf_den;
  Position glob_point;
  Index ind;

  std::string tagLayerName;

  tagLayerName = getTagLayerName(i);
  Size siz = _rfid_belief_maps.getSize();
  rxPw_mat = Eigen::MatrixXf(siz(0), siz(1));
  double orientation_rad = orientation_deg * M_PI/180.0;

  for (grid_map::GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd(); ++iterator) {
      // matrix indexes...
      ind = *iterator;      
      // get cell center of the cell in the map frame.            
      _rfid_belief_maps.getPosition(ind, glob_point);
      // that is where the tag is in map coordinates
      tag_x = glob_point.x();
      tag_y = glob_point.y();

      // now get robot - tag relative pose!
      double delta_x = (tag_x - x_m ); 
      double  delta_y = (tag_y - y_m);
      // rotate
      tag_x =  delta_x * cos(orientation_rad) + delta_y * sin(orientation_rad);
      tag_y = -delta_x * sin(orientation_rad) + delta_y * cos(orientation_rad);

      getSphericCoords(tag_x,tag_y, tag_r, tag_h);

      rxP = received_power_friis_polar(tag_r, tag_h,freq, _txtPower, _antenna_gains);
      rxPw_mat(ind(0),ind(1))  = rxP;              
  }
  // get pdf to all points: likelihood ..................................................................
        
  // gaussian pdf
  // fddp(x,mu,sigma) = exp( -0.5 * ( (x - mu)/sigma )^2 )   /   (sigma * sqrt(2 pi )) 
  likl_mat = rxPw_mat;
  pdf_den = _sigma_power * sqrt( 2.0 * M_PI) ;
  likl_mat = (rxPower - likl_mat.array())/_sigma_power;
  likl_mat =  likl_mat.array().pow(2.0);
  likl_mat = -0.5 * likl_mat;
  likl_mat = likl_mat.array().exp()/pdf_den;
  
  // trick: if obstacles is binary, this would have 1 on free space and 0 in obstacles
  Eigen::MatrixXf obst_mat = _rfid_belief_maps["ref_map"];
  obst_mat = obst_mat/_free_space_val;

  // this should remove prob at obstcles
  likl_mat = likl_mat.cwiseProduct(obst_mat);
  // normalize in this space:
  double bayes_den = likl_mat.sum();
  // _rfid_belief_maps.add("temp", 0.0);
  // _rfid_belief_maps["temp"] = likl_mat;
  // double bayes_den = getNormalizingFactorBayesFullMap(x_m, y_m, orientation_rad, tagLayerName);
  
  if (bayes_den>0){
      likl_mat = likl_mat/bayes_den;
      // now do bayes ...  everywhere
      _rfid_belief_maps[tagLayerName] = _rfid_belief_maps[tagLayerName].cwiseProduct(likl_mat);
      // Normalise
      // _rfid_belief_maps[tagLayerName] = _rfid_belief_maps[tagLayerName]/bayes_den;
  }
  normalizeRFIDLayer(tagLayerName);
}

GridMap RadarModel::getActiveAreaMaps(){
  return this->_active_area_maps;
}

GridMap RadarModel::getBeliefMaps(){
  return this->_rfid_belief_maps;
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
        if (isnan(likelihood)) likelihood = 0.0;
        neg_likelihood = 1 - likelihood;
        if (isnan(neg_likelihood)) neg_likelihood = 0.0;
        
        log2_likelihood = log2(likelihood);
        if (isinf(log2_likelihood)) log2_likelihood = 0.0;
        log2_neg_likelihood = log2(neg_likelihood);
        if (isinf(log2_neg_likelihood)) log2_neg_likelihood = 0.0;
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

double RadarModel::getTotalKL(double x, double y, double orientation,
                                  grid_map::SubmapIterator iterator,
                                  int tag_i) {

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
        tmp_KL = posterior * log(posterior/prior);
        if (isnan(tmp_KL)) tmp_KL = 0;
        total_KL += tmp_KL;
        // cout << "Prior: " << prior << endl; 
        // cout << "Posterior: " << posterior << endl;
        // cout << "totalKL: " << total_KL << endl;
      }
    }
  }
  return total_KL;
}


float RadarModel::getFreeSpaceVal(){
  return this->_free_space_val;
}


void RadarModel::cutPowerBasedObstacleDistribution(double* rxPower, Pose* target, std::pair<int, int> relTagCoord){
  // Check if there is line of sight between antenna and tag
  grid_map::Index start;
  grid_map::Index end;
  this->getBeliefMaps().getIndex(Position( target->getX(),  target->getY()),start);
  this->getBeliefMaps().getIndex(Position( relTagCoord.first,  relTagCoord.second),end);
  cout << "R: [" << target->getX() << "," << target->getY() << "], T: [" << relTagCoord.first << "," << relTagCoord.second << "]" << endl;

  // If there is an obstacles along the line antenna-tag, cut the rxPower below the sensitivity
  int count_obs_cell = 0;
  int count_free_cell = 0;
  // TODO: Check that start and end are correct
  for (grid_map::LineIterator iterator(this->getBeliefMaps(), start, end); !iterator.isPastEnd(); ++iterator) {
    if (( this->getBeliefMaps().at("ref_map", *iterator) == this->getFreeSpaceVal()  )){
      count_free_cell++;
    } else count_obs_cell++;
  }
  // cout << count_obs_cell << " : " << count_free_cell << endl;
  if (count_obs_cell >= 10) *rxPower = *rxPower * 6 ;
}