#include <map.h>
#include <fstream>
#include <iostream>
using namespace std;

namespace dummy{

/**
 * @brief Map::Map constructor
 * @param infile: the path used for reading the map
 * @param fileURI: map file absolute path and filename
 * @param resolution: the resolution of the map
 * @param imgresolution: the resolution for building the planningGrid
 */
Map::Map(std::ifstream& infile, double resolution, double imgresolution)
{

  Map::createMap(infile);
  Map::createGrid(resolution);
  Map::createPathPlanningGrid(imgresolution);
  Map::createNewMap();
}

/**
 * @brief Map::Map empty constructore
 */
Map::Map()
{

}

/**
 * @brief Map::createMap create a monodimensional vector containing the map
 * @param infile: the path for reading the map
 */
void Map::createMap(std::ifstream& infile)
{
  long row = 0, col = 0;
  std::stringstream ss;
  std::string inputLine = "";

  // First line : version
  getline(infile,inputLine);
  if(inputLine.compare("GRID") == 0)
  {
    int temp;
    infile >> Map::numRows >> Map::numCols;
    //std::cout << "numrows " << numRows << " numcols " << numCols << std::endl;
    Map::map.reserve(numRows*numCols);
    std::cout << map.size() << std::endl;
    for(int i = 0; i < numRows*numCols; ++i)
    {
      infile >> temp;
      if(temp == 1)
        map.push_back(0);
      if(temp == 0)
        map.push_back(255);
    }

  }
  else
  {
    if(inputLine.compare("P2") != 0)
    {
      long size, greylevels;
      //Second line : comment
      char comment_char = infile.get();
      if(comment_char == '#')
      {
        getline(infile,inputLine);
      }
      else
      {
        //cout << "No comment in the header" << endl;
        ss << comment_char;
      }
      // Continue with a stringstream
      ss << infile.rdbuf();
      // Third line : size
      ss >> numCols >> numRows >> greylevels;
      // cout << numCols << " columns and " << numRows << " rows" << endl;
      getline(infile,inputLine);
      size = numCols * numRows;
      long* data = new long[size];
      for(long* ptr = data; ptr < data+size; ptr++)
      {
        // read in binary char
        unsigned char t_ch = ss.get();
        // convert to integer
        long t_data = static_cast<long>(t_ch);
        // if passes add value to data array
        *ptr = t_data;
      }
      // close the stream
      infile.close();

      Map::map.reserve(numRows*numCols);
      // Following lines : data
      for(long* ptr = data, i = 0; ptr < data+size; ptr++, i++)
      {
        map[i] = *ptr ;
        //cout << map[i];
      }
      delete data;

    }
    else
    {
      //Second line : comment
      char comment_char = infile.get();
      if(comment_char == '#') {
        getline(infile,inputLine);
      } else {
        //cout << "No comment in the header" << endl;
        ss << comment_char;
      }
      //Following lines not useful anymore
      //getline(infile,inputLine);
      //cout << "Comment : " << inputLine << endl;
      // Continue with a stringstream
      ss << infile.rdbuf();
      // Third line : size
      ss >> numCols >> numRows;
      // cout << numCols << " columns and " << numRows << " rows" << endl;
      //max value of color
      //getline(infile,inputLine);
      int max;
      ss >> max;
      //std::vector<int> array(numRows*numCols);
      Map::map.reserve(numRows*numCols);
      // Following lines : data
      for(row = 0; row < numRows; ++row)
        for (col = 0; col < numCols; ++col)
        {
          ss >> Map::map[row*numCols + col];
        }
      //infile.close();
    }

  }
}



/**
 * @brief Map::createGrid create the map as a monodimension vector with 0 and 1
 * @param resolution: the size of the cell's side
 */
void Map::createGrid(double resolution)
{
  //cluster cells into grid
  float clusterSize = static_cast<float>(1/resolution);
  Map::numGridRows = static_cast<long>(numRows/clusterSize);
  Map::numGridCols = static_cast<long>(numCols/clusterSize);
  // cout <<" numGridRows: " << numGridRows <<", numGridCols: "<< numGridCols << endl;

  for(int i = 0; i < numGridCols*numGridRows; ++i)
  {
    grid.push_back(0);
  }

  //set 1 in the grid cells corrisponding to obstacles
  for(long row = 0; row < numRows; ++row)
  {
    for(long col = 0; col < numCols; ++col)
    {

      // If the value in the map is below 250, set it to 1 to represent a free cell
      if(map[row*numCols + col] < 250)
      {
        grid[static_cast<long>((row/clusterSize)*numGridCols) + static_cast<long>((col/clusterSize))] = 1;
        //NOTE: i don't remember when it should be used
        //map[(long)(row/clusterSize)*numGridCols + (long)(col/clusterSize)] = 1;
      }
    }
  }


}

/**
 * @brief Map::createNewMap save on disk an image representing the map of
 * the environment and a list of free cells in the map
 */
void Map::createNewMap()
{
  std::ofstream imgNew("/home/pulver/Desktop/MCDM/input_map.pgm", ios::out);
  std::ofstream txt("/home/pulver/Desktop/MCDM/freeCell.txt");
  long columns = numGridCols;
  long rows = numGridRows;

  imgNew << "P2\n" << columns << " " << rows << "\n255\n";

  for(long row = 0; row < rows; ++row)
  {
    for(long col = 0; col < columns; ++col)
    {
      //if an obstacle is present put 255 as black
      if(getGridValue(row,col) == 0) {
        imgNew<<  255 << " ";

      }
      //else put 0 as free cell
      else {
        imgNew <<  0 << " ";
        txt <<  col << ": " << row << endl;
      }
    }
    //imgNew << "\n";
  }

  imgNew.close();
  txt.close();
}

/**
 * Map::createPathPlanningGrid Create the gridmap for path planning and for radio sensing (both are identical)
 * @param  resolution: the size of the side of a cell
 * @return          nothing
 */
void Map::createPathPlanningGrid(double resolution)
{
  //cluster cells into grid
  float clusterSize = static_cast<float>(1/resolution);
  std::cout << "imgResolution: " << resolution << " clusterSize: " << clusterSize << std::endl;
  std::cout << "numRows: " << numRows << " numCols: " << numCols << std::endl;
  Map::numPathPlanningGridRows = static_cast<int>(numRows/clusterSize);
  Map::numPathPlanningGridCols = static_cast<int>(numCols/clusterSize);
  cout <<"numPathPlanningGridRows: " << numPathPlanningGridRows <<", numPathPlanningGridCols: "<< numPathPlanningGridCols << endl;

  for(int i = 0; i < numPathPlanningGridCols*numPathPlanningGridRows; ++i)
  {
    pathPlanningGrid.push_back(0);
    RFIDGrid.push_back(250);
  }

  //set 1 in the grid cells corrisponding to obstacles
  for(long row = 0; row < numRows; ++row)
  {
    for(long col = 0; col < numCols; ++col)
    {

      if(map[row*numCols + col] < 250)
      {
        pathPlanningGrid[static_cast<long>((row/clusterSize)*numPathPlanningGridCols) + static_cast<long>(col/clusterSize)] = 1;
        RFIDGrid[static_cast<long>((row/clusterSize)*numPathPlanningGridCols) + static_cast<long>(col/clusterSize)] = 1;
        //NOTE: i don't remember when it should be used
        //map[(long)(row/clusterSize)*numGridCols + (long)(col/clusterSize)] = 1;
      }else {
        listFreeCells.push_back(std::make_pair((row/clusterSize), col/clusterSize));
      }
    }
  }
  Map::gridToPathGridScale = static_cast<int>(numGridRows / numPathPlanningGridRows);
  // cout << "Scale: " << gridToPathGridScale << endl;


}


void Map::updatePathPlanningGrid(int posX, int posY, int rangeInMeters, double power)
{

  //int ppX = (int)(posX/gridToPathGridScale);
  //int ppY = (int)(posY/gridToPathGridScale);
  int minX = posX - rangeInMeters;
  int maxX = posX + rangeInMeters;
  if(minX < 0) minX = 0;
  if(maxX > numPathPlanningGridRows - 1) maxX = numPathPlanningGridRows - 1;
  int minY = posY - rangeInMeters;
  int maxY = posY + rangeInMeters;
  if(minY < 0) minY = 0;
  if(maxY > numPathPlanningGridCols - 1) maxY = numPathPlanningGridCols - 1;

  for(int row = minX; row <= maxX; ++row)
  {
    for(int col = minY; col <= maxY; ++col)
    {
      int countScanned = 0;
      int setToOne = 0;
      for(int gridRow = row*gridToPathGridScale; gridRow < row*gridToPathGridScale + gridToPathGridScale; ++gridRow)
      {
        for(int gridCol = col*gridToPathGridScale; gridCol < col*gridToPathGridScale + gridToPathGridScale; ++gridCol)
        {
          // if the cell contains an obstacle
          if(getGridValue(gridRow, gridCol) == 1)
          {
            setToOne = 1;
          }

          // if the cell is free and scanned
          if(getGridValue(gridRow, gridCol) == 2)
          {
            countScanned++;
          }
        }
      }
      if(countScanned == gridToPathGridScale*gridToPathGridScale)
      {
        setPathPlanningGridValue(2, row, col);
        setRFIDGridValue(power, row, col);
      }
      if(setToOne == 1) setPathPlanningGridValue(1, row, col);
    }
  }

}



/**
 * @brief Map::getPathPlanningGridValue get the current value of a cell in the planning grid
 * @param i: the x-position(row) in the planning grid
 * @param j: the y-position(column) in the planning grid
 * @return the value in that cell value: 0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell)
 */
int Map::getPathPlanningGridValue(long i,long j) const
{
  return pathPlanningGrid[i*numPathPlanningGridCols + j];
}

/**
 * @brief Map::setPathPlanningGridValue assign a value to a cell in the planning grid
 * @param value:  0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell
 * @param i: the x-position(row) of the cell in the planning grid
 * @param j: the y-position(column) of the cell in the planning grid
 */
void Map::setPathPlanningGridValue(int value, int i, int j)
{
  pathPlanningGrid[i*numPathPlanningGridCols + j] = value;
}

/**
 * @brief Map::setRFIDGridValue: update the grid cell summing the current value with a new reading
 * @param power: the sensed power by the antenna
 * @param i: the x-position in the grid
 * @param j: the y-position in the grid
 */
void Map::setRFIDGridValue(float power, int i, int j)
{
  //  cout << "-----" << endl;
//  if( power < 0) power = 0;
//    cout << RFIDGrid[i * numPathPlanningGridCols + j] << endl;
  RFIDGrid[i * numPathPlanningGridCols + j] += power;
//    cout << RFIDGrid[i * numPathPlanningGridCols + j] << endl;
}

/**
 * @brief Map::getPathPlanningNumCols get the number of columns in the planning grid
 * @return
 */
int Map::getPathPlanningNumCols() const
{
  return numPathPlanningGridCols;
}

/**
 * @brief Map::getPathPlanningNumRows get the number of rows in the planning grid
 * @return
 */
int Map::getPathPlanningNumRows() const
{
  return numPathPlanningGridRows;
}

/**
 * @brief Map::getGridToPathGridScale get the scale(ratio) between the cell size
 * in the grid and in that used for planning
 * @return the scale value
 */
int Map::getGridToPathGridScale() const
{
  return gridToPathGridScale;
}

/**
 * @brief Map::setGridValue set a value for a cell in the Grid
 * @param value: 0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell
 * @param i: the x-position(row) in the grid
 * @param j: the y-position(column) in the grid
 */
void Map::setGridValue(int value, long i, long j)
{
  if(value == 0 || value == 1 || value == 2)
  {
    grid[i*numGridCols + j] = value;
  }
}

void Map::addEdgePoint(int x, int y)
{
  std::pair<int,int> pair(x,y);
  edgePoints.push_back(pair);
}

/**
 * @brief Map::getMap2D create a 2D map from a vector one
 * @return the 2D map
 */
std::vector<vector<long> > Map::getMap2D(){
  vector<vector<long> > map2D;

  for (long gridRow = 0; gridRow < numGridRows; ++gridRow)
  {
    for (long gridCol = 0; gridCol < numGridCols; ++gridCol)
    {
      map2D[gridRow].at(gridCol) = getGridValue(gridRow,gridCol);
    }
  }

  return map2D;

}

//various getters
/**
 * @brief Map::getGridValue get the value associated with one cell of the grid
 * @param i: the x-position in the grid
 * @param j: the y-position in the grid
 * @return the associated value with that cell
 */
int Map::getGridValue(long i,long j) const
{
  return grid[i*numGridCols + j];
}

/**
 * @brief Map::getRFIDGridValue get the value(power) associated with one cell of the RFIDgrid
 * @param i: the x-position in the RFID grid
 * @param j: the y-position in the RFID grid
 * @return the associated value with that cell
 */
int Map::getRFIDGridValue(long i,long j) const
{
  return RFIDGrid[i * numPathPlanningGridCols + j];
}

/**
 * @brief Map::getMapValue get the value of one cell of the map
 * @param i: the x-position in the map
 * @param j: the y-position in the map
 * @return the value associated with that cell
 */
long Map::getMapValue(long i, long j)
{
  return map[i*numCols + j];
}

/**
 * @brief Map::getNumRows get the number of columns of the grid
 * @return the number of columns of the grid
 */
long Map::getNumGridCols() const
{
  return numGridCols;
}

/**
 * @brief Map::getNumRows get the number of rows of the grid
 * @return the number of rows of the grid
 */
long Map::getNumGridRows() const
{
  return numGridRows;
}

/**
 * @brief Map::getNumRows get the number of columns of the map
 * @return the number of columns of the map
 */
long Map::getNumCols()
{
  return numCols;
}

/**
 * @brief Map::getNumRows get the number of rows of the map
 * @return the number of rows of the map
 */
long Map::getNumRows()
{
  return numRows;
}

/**
 * @brief dummy::Map::getGridValue get the value of a cell in the gridmap
 * @param i: the index of the cell
 * @return the value contained in the cell
 */
long dummy::Map::getGridValue(long i) const
{
  return Map::grid[i];
}

/**
 * @brief Map::getRobotPosition get the current pose of the robot
 * @return the current pose of the robot
 */
Pose Map::getRobotPosition()
{
  return currentPose;
}

/**
 * @brief Map::setCurrentPose set the pose of the robot in the map
 * @param p the current pose of the robot
 */
void Map::setCurrentPose(Pose& p)
{
  currentPose = p;
}

/**
 * @brief Map::getTotalFreeCells calculate the number of free cells in the map
 * @return the number of free cells
 */
long Map::getTotalFreeCells(){

  totalFreeCells = numGridCols * numGridRows;
  for(long row = 0; row < numGridRows; ++row)
  {
    for(long col = 0; col < numGridCols; ++col)
    {
      //if an obstacle is present decrement the number of free cells
      if(getGridValue(row,col) == 1) totalFreeCells--;
    }
  }
  //cout << "Total free cells: " << totalFreeCells << endl;
  return totalFreeCells;
}

/** * @brief Map::decreaseFreeCells decrease the number of the cells that still must be covered
 */
void Map::decreaseFreeCells(){
  this->totalFreeCells--;
}

/**
 * @brief Map::drawVisitedCells save on disk an image of the map
 */
void Map::drawVisitedCells()
{
  std::ofstream resultMap("/home/pulver/Desktop/MCDM/result_free_cells.pgm", ios::out);
  long columns = numGridCols;
  long rows = numGridRows;

  resultMap << "P2\n" << columns << " " << rows << "\n255\n";

  for(long row = 0; row < rows; ++row)
  {
    for(long col = 0; col < columns; ++col)
    {
      //string encoding = to_string(col) + to_string(row);
      //if an obstacle is present put 255 as black
      if(getGridValue(row,col) == 0) {
        resultMap<<  255 << " ";
      }
      //else put 0 as free cell
      else {
        resultMap <<  0 << " ";

      }
    }
    //imgNew << "\n";
  }

  resultMap.close();

}

/**
 * @brief Map::drawRFIDScan save on disk an image representing the scanned environment. Lighter zone represents
 * higher probability for the presence of the RFID tag
 */
void Map::drawRFIDScan()
{
  std::ofstream resultMap("/home/pulver/Desktop/MCDM/rfid_result.pgm", ios::out);
  long columns = numPathPlanningGridCols;
  long rows = numPathPlanningGridRows;

  resultMap << "P2\n" << columns << " " << rows << "\n255\n";

  for(long row = 0; row < rows; ++row)
  {
      for(long col = 0; col < columns; ++col)
      {
          int value = getRFIDGridValue(row, col);
//      cout << value << endl;
          value = std::min(value, 255);
          value = std::max(value, 0);
          resultMap << value  << " ";
      }
  }

  resultMap.close();
}

/**
 * @brief Map::drawRFIDScan save on disk an image representing the scanned environment. Lighter zone represents
 * higher probability for the presence of the RFID tag
 */
void Map::drawRFIDGridScan(RFIDGridmap grid)
{
  std::ofstream resultMap("/home/pulver/Desktop/MCDM/rfid_grid_scan.pgm", ios::out);
  long columns = numPathPlanningGridCols;
  long rows = numPathPlanningGridRows;

  resultMap << "P2\n" << columns << " " << rows << "\n255\n";

  for(long row = 0; row < rows; ++row)
  {
    for(long col = 0; col < columns; ++col)
    {
      int value = static_cast<int>(grid.getCell(row, col));
//     cout << value << endl;
      value = std::min(value, 255);
      value = std::max(value, 0);
      if (value <= 200) value = std::max(value - 100, 0);
      resultMap << value  << " ";
    }
  }

  resultMap.close();
}
/**
 * @brief Map::printVisitedCells save on disk the list of all the cells visited by the robot
 * @param history: the list of visited cells
 */
void Map::printVisitedCells(vector< string >& history)
{
  std::ofstream txt("/home/pulver/Desktop/MCDM/finalResult.txt");
  for (int i =0 ; i < history.size(); i++){
    string encoding = history.at(i);
    string s ;
    stringstream ss;
    ss << s;
    char delimiter('/');
    string x,y,orientation,r,phi;
    std::string::size_type pos = encoding.find(delimiter);
    x = encoding.substr(0,pos);
    int xValue = atoi(x.c_str());
    //cout << x << endl;
    string newString = encoding.substr(pos+1,encoding.size());
    //cout << newString << endl;
    std::string::size_type newPos = newString.find(delimiter);
    y = newString.substr(0,newPos);
    int yValue = atoi(y.c_str());
    //cout << y << endl;
    newString = newString.substr(newPos+1,encoding.size());
    //cout << newString << endl;
    newPos = newString.find(delimiter);
    orientation = newString.substr(0,newPos);
    int orientationValue = atoi(orientation.c_str());
    //orientationValue = 180 - orientationValue;
    //cout << orientation << endl;
    newString = newString.substr(newPos+1,encoding.size());
    //cout << newString << endl;
    newPos = newString.find(delimiter);
    r = newString.substr(0,newPos);
    int rValue = atoi(r.c_str());
    //cout << r << endl;
    newString = newString.substr(newPos+1,encoding.size());
    //cout << newString << endl;
    newPos = newString.find(delimiter);
    phi = newString.substr(0,newPos);
    double phiValue = atoi(phi.c_str());

    txt << xValue << "  " << yValue << " " << orientationValue << " " << r <<endl;
  }

}

/**
 * @brief Map::getRelativeTagCoord calculate the relative position of the RFID tag wrt the antenna
 * @param absTagX: absolute x-position of the RFID tag
 * @param absTagY: absolute x-position of the RFID tag
 * @param antennaX: absolute x-position of the antenna
 * @param antennaY: absolute y-position of the antenna
 * @return a pair containing the relative position of the tag to the antenna
 */
std::pair<int, int> Map::getRelativeTagCoord(int absTagX, int absTagY, int antennaX, int antennaY)
{
//  cout << "TAG = [" << absTagX << "," << absTagY <<"] -- ANTENNA = [" << antennaX << "," << antennaY << "]" << endl;
  std::pair<int, int> relTagCoord;
  relTagCoord.first = std::abs(absTagX - antennaX);
  relTagCoord.second = std::abs(absTagY - antennaY);
//  cout << "RELTAG = [" << relTagCoord.first << "," << relTagCoord.second << "]" << endl;
  return relTagCoord;
}


std::pair<int, int> Map::findTag()
{
  std::pair<int,int> tag(0,0);
  double powerRead = 0.0;
  for(long row = 0; row < numPathPlanningGridRows; row++)
  {
    for(long col = 0; col < numPathPlanningGridCols ; col++)
    {

      // Instead at looking at the individual pixel, sum the power in the 5x5 surrounding area
      double local_power = 0.0;
      for (int i = -5; i <= 5; i++){
        for (int j = -5; j <= 5; j++){
          int value = (getRFIDGridValue(row + i, col + j));
          // cout << value << endl;
          // value = std::min(value, 255);
          // value = std::max(value, 0);
          // if (value != 0) value = 255 - value;
          local_power = local_power + value;
          // local_power = local_power + getRFIDGridValue(row + i, col + j);
        }
      }
      // cout << "local_power: " << local_power << endl;
      if(local_power < powerRead)
      {
        powerRead = local_power;
        // cout << "Value read: " << powerRead << endl;
        tag.first = row;
        tag.second = col;
      }
    }
  }
  return tag;
}

std::pair<int, std::pair<int, int>> Map::findTagfromGridMap(RFIDGridmap grid)
{
  std::pair<int,int> tag(0,0);
  double powerRead = 0;
  for(int row=0; row < numPathPlanningGridRows; row++)
  {
    for(int col=0; col < numPathPlanningGridCols; col++)
    {
      double tmp_power = 0.0;
      // for (int i = -3; i <= 3; i++){
      //   for (int j = -3; j <= 3; j++){
      //     tmp_power = tmp_power + grid.getCell(row, col); 
      //   }
      // }
      tmp_power = grid.getCell(row, col); 
      if(tmp_power > powerRead)
      {
        powerRead = tmp_power;
        
        tag.first = row;
        tag.second = col;
      }

//       if(grid.getCell(row, col) > powerRead)
//       {
//         powerRead = getRFIDGridValue(row, col);
// //        cout << "Value read: " << powerRead << endl;
//         tag.first = row;
//         tag.second = col;
//       }
    }
  }
  std::pair<int, std::pair<int, int>> final_return (powerRead, tag);
  // cout << "Value read: " << powerRead << endl;
  return final_return;
}

std::pair<long, long> Map::getRandomFreeCell(){
  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(0, totalFreeCells-1); // guaranteed unbiased

  auto random_integer = uni(rng);
  std::pair<long, long> random_cell = listFreeCells.at(random_integer);
  return random_cell;
}

}
