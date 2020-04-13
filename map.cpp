#include <map.h>
#include <fstream>
#include <iostream>
#include <random>
using namespace std;

namespace dummy{


Map::Map(std::ifstream& infile, double resolution, double imgresolution)
{

  Map::createMap(infile);
  Map::createGrid(resolution);
  Map::createPathPlanningGrid(imgresolution);
  Map::createNewMap();
}

Map::Map()
{
}

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

void Map::createPathPlanningGrid(double resolution)
{
  //cluster cells into grid
  float clusterSize = static_cast<float>(1/resolution);
  // std::cout << "imgResolution: " << resolution << " clusterSize: " << clusterSize << std::endl;
  // std::cout << "numRows: " << numRows << " numCols: " << numCols << std::endl;
  Map::numPathPlanningGridRows = static_cast<int>(numRows/clusterSize);
  Map::numPathPlanningGridCols = static_cast<int>(numCols/clusterSize);
  // cout <<"numPathPlanningGridRows: " << numPathPlanningGridRows <<", numPathPlanningGridCols: "<< numPathPlanningGridCols << endl;

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

int Map::getPathPlanningGridValue(long i,long j) const
{
  return pathPlanningGrid[i*numPathPlanningGridCols + j];
}

void Map::setPathPlanningGridValue(int value, int i, int j)
{
  pathPlanningGrid[i*numPathPlanningGridCols + j] = value;
}

void Map::setRFIDGridValue(float power, int i, int j)
{
  //  cout << "-----" << endl;
  //  if( power < 0) power = 0;
  //    cout << RFIDGrid[i * numPathPlanningGridCols + j] << endl;
  RFIDGrid[i * numPathPlanningGridCols + j] += power;
  //    cout << RFIDGrid[i * numPathPlanningGridCols + j] << endl;
}

int Map::getPathPlanningNumCols() const
{
  return numPathPlanningGridCols;
}

int Map::getPathPlanningNumRows() const
{
  return numPathPlanningGridRows;
}

int Map::getGridToPathGridScale() const
{
  return gridToPathGridScale;
}


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

int Map::getGridValue(long i,long j) const
{
  return grid[i*numGridCols + j];
}


int Map::getRFIDGridValue(long i,long j) const
{
  return RFIDGrid[i * numPathPlanningGridCols + j];
}

long Map::getMapValue(long i, long j)
{
  return map[i*numCols + j];
}

long Map::getNumGridCols() const
{
  return numGridCols;
}

long Map::getNumGridRows() const
{
  return numGridRows;
}


long Map::getNumCols()
{
  return numCols;
}

long Map::getNumRows()
{
  return numRows;
}

long dummy::Map::getGridValue(long i) const
{
  return Map::grid[i];
}

Pose Map::getRobotPosition()
{
  return currentPose;
}

void Map::setCurrentPose(Pose& p)
{
  currentPose = p;
}

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

void Map::decreaseFreeCells(){
  this->totalFreeCells--;
}

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
  int value = 0;
  for(long row = 0; row < numPathPlanningGridRows; row++)
  {
    for(long col = 0; col < numPathPlanningGridCols ; col++)
    {

      // Instead at looking at the individual pixel, sum the power in the 5x5 surrounding area
      double local_power = 0.0;
      for (int i = -5; i <= 5; i++){
        for (int j = -5; j <= 5; j++){
          value = (getRFIDGridValue(row + i, col + j));
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
  int buffer_size = 3;
  for(int row=0; row < numPathPlanningGridRows; row++)
  {
    for(int col=0; col < numPathPlanningGridCols; col++)
    {
      double tmp_power = 0.0;
      for (int i = -buffer_size; i <= buffer_size; i++){
        for (int j = -buffer_size; j <= buffer_size; j++){
          tmp_power = tmp_power + grid.getCell(row, col); 
        }
      }
      // tmp_power = grid.getCell(row, col); 
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
