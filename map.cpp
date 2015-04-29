#include <map.h>
using namespace std;

namespace dummy{
  
Map::Map(std::ifstream& infile, int resolution)
{
  Map::createMap(infile);
  Map::createGrid(resolution);
  Map::createNewMap();
}

Map::Map()
{
  
}



// create a monodimensional vector containing the map 
void Map::createMap(std::ifstream& infile)
{
  int row = 0, col = 0;
  std::stringstream ss;
  std::string inputLine = "";

  // First line : version
  getline(infile,inputLine);
  //if(inputLine.compare("P2") != 0) cerr << "Version error" << endl;
  //else cout << "Version : " << inputLine << endl;

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
  
 // Second line : comment
 getline(infile,inputLine);
 //cout << "Comment : " << inputLine << endl;
 
 //std::vector<int> array(numRows*numCols);
 Map::map.reserve(numRows*numCols);

  // Following lines : data
  for(row = 0; row < numRows; ++row)
    for (col = 0; col < numCols; ++col) ss >> Map::map[row*numCols + col];
 
  infile.close();
}

// get the map as a monodimension vector with 0 and 1
void Map::createGrid(int resolution)
{
  //cluster cells into grid
  int clusterSize = (int)((double)(100/resolution));
  Map::numGridRows = (int)numRows/clusterSize;
  Map::numGridCols = (int)numCols/clusterSize;
  int gridRow = 0, gridCol = 0;

  //cout << "Total cell: " << totalFreeCells << endl;
  
  //get the size of the array and initialize to 0
  
  Map::grid.reserve(numGridRows*numGridCols);
  
  for (gridRow = 0; gridRow < numGridRows; ++gridRow)
  {
    for (gridCol = 0; gridCol < numGridCols; ++gridCol)
    {
      grid[gridRow*numGridCols + gridCol] = 0;
    }
  }
  
  //set 1 in the grid cells corrisponding to obstacles
  for(int row = 0; row < numRows; ++row)
  {
    for(int col = 0; col < numCols; ++col)
    {
	//if(map[row*numCols + col] == 0) 
      if(map[row*numCols + col] < 210) 
      {
	    grid[(int)((float)row/clusterSize)*numGridCols + (int)((float)col/clusterSize)] = 1;
	    
      }
    }
  }
}

void Map::createNewMap()
{
    std::ofstream f("/home/pulver/Desktop/test.pgm");
    std::ofstream txt("/home/pulver/Desktop/freeCell.txt");
    int columns = getNumGridCols();
    int rows = getNumGridRows();
    
    f << "P2\n" << columns << " " << rows << "\n255\n";
    
    for(int row = 0; row < rows; ++row)
    {
	for(int col = 0; col < columns; ++col)
	{
	    //if an obstacle is present put 255 as black
	    if(getGridValue(row,col) == 0) {
		f <<  255 << " ";
	    txt <<  col << ": " << row << endl;
	    }
	    //else put 0 as free cell
	    else {
		f <<  0 << " ";
		
	    }
	}
	f << "\n";
    }
}



// values: 0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell
void Map::setGridValue(int value, int i, int j)
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

  
std::vector<vector<int> > Map::getMap2D(){
    vector<vector<int> > map2D;
    
    for (int gridRow = 0; gridRow < numGridRows; ++gridRow)
  {
    for (int gridCol = 0; gridCol < numGridCols; ++gridCol)
    {
      map2D[gridRow].at(gridCol) = getGridValue(gridRow,gridCol);
    }
  }
    
    return map2D;
    
}

//various getters

int Map::getGridValue(int i, int j) const
{
  return grid[i*numGridCols + j];
}

int Map::getMapValue(int i, int j)
{
  return map[i*numCols + j];
}


int Map::getNumGridCols() const
{
  return numGridCols;
}

int Map::getNumGridRows() const
{
  return numGridRows;
}

int Map::getNumCols()
{
  return numCols;
}

int Map::getNumRows()
{
  return numRows;
}

int dummy::Map::getGridValue(int i) const
{
  return Map::grid[i];
}

Pose Map::getRobotPosition()
{
 
    int x,y,orientation,range;
    double FOV;
    x = rand() % 100;
    y = rand() % 100;
    orientation = 0;
    range = rand() % 30;
    FOV = 45;
    Pose p = Pose(x,y,orientation,range,FOV);
    return p;
}

long Map::getTotalFreeCells(){
    int columns = getNumGridCols();
    int rows = getNumGridRows();
    totalFreeCells = columns * rows;
    for(int row = 0; row < rows; ++row)
    {
	for(int col = 0; col < columns; ++col)
	{
	    //if an obstacle is present put 255 as black
	    if(getGridValue(row,col) == 1) totalFreeCells--;
	}
    }
    //cout << "Total free cells: " << totalFreeCells << endl;
    return totalFreeCells;
}

void Map::decreaseFreeCells(){
    this->totalFreeCells--;
}
}