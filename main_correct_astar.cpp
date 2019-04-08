#include <algorithm>
#include <iostream>
#include <iterator>
#include "map.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "Criteria/traveldistancecriterion.h"
#include "radio_models/propagationModel.cpp"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>
#include <time.h>
#include <ctime>



using namespace std;
using namespace dummy;
bool contains ( std::list< Pose >& list, Pose& p );
void cleanPossibleDestination2 ( std::list< Pose > &possibleDestinations, Pose& p );
void pushInitialPositions ( Map map, int x, int y, int orientation,  int range, int FOV, double threshold,
                            string actualPose, vector< pair< string, list< Pose > > > *graph2 );
double calculateScanTime ( double scanAngle );
Pose createFromInitialPose ( int x, int y, int orientation, int variation, int range, int FOV );

// Input : ./mcdm_online_exploration_ros ./../Maps/map_RiccardoFreiburg_1m2.pgm 100 75 5 0 15 180 0.95 0.12
// resolution x y orientation range centralAngle precision threshold
int main ( int argc, char **argv )
{
  auto startMCDM = chrono::high_resolution_clock::now();
  ifstream infile;
  infile.open ( argv[1] );  // the path to the map
  double resolution = atof ( argv[2] );  // the resolution of the map
  double imgresolution = atof ( argv[10] );  // the resolution to use for the planningGrid and RFIDGrid
  Map map = Map ( infile,resolution, imgresolution );
  cout << "Map dimension: " << map.getNumGridCols() << " : "<<  map.getNumGridRows() << endl;
  int gridToPathGridScale = map.getGridToPathGridScale();
  // i switched x and y because the map's orientation inside and outside programs are different
  long initX = static_cast<long>( atoi ( argv[4] ) *imgresolution );  // initial X-position of the robot in map frame
  long initY = static_cast<long>( atoi ( argv[3] ) *imgresolution );  // initial Y-position of the robot in map frame
  std::cout << "initX: " << initX << " initY: " << initY << std::endl;
  int initOrientation = atoi ( argv[5] );  // initial orientation of the robot in map frame
  double initFov = atoi ( argv[7] );  // initial FOV of the robot sensor
  initFov = initFov * PI /180;
  int initRange = atoi ( argv[6] );
  double precision = atof ( argv[8] );
  double threshold = atof ( argv[9] );
  //x,y,orientation,range,FOV

  Pose initialPose = Pose ( initX,initY,initOrientation,initRange,initFov );
  Pose invertedInitial = createFromInitialPose ( initX,initY,initOrientation,180,initRange,initFov );
  Pose eastInitial = createFromInitialPose ( initX,initY,initOrientation,90,initRange,initFov );
  Pose westInitial = createFromInitialPose ( initX,initY,initOrientation,270,initRange,initFov );
  Pose target = initialPose;
  Pose previous = initialPose;
  long numConfiguration = 0;
  vector<pair<string,list<Pose>>> graph2;
  NewRay ray;
  ray.setGridToPathGridScale ( gridToPathGridScale );
  MCDMFunction function;
  long sensedCells = 0;
  long newSensedCells =0;
  long totalFreeCells = map.getTotalFreeCells();
  int count = 0;
  double travelledDistance = 0;
  int numOfTurning = 0;
  unordered_map<string,int> visitedCell;
  vector<string>history;
  history.push_back ( function.getEncodedKey ( target,1 ) );
  //amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
  unsigned int microseconds = 5 * 1000 * 1000 ;
  list<Pose> unexploredFrontiers;
  list<Pose> tabuList;
  tabuList.push_back(target);
  list<Pose> nearCandidates;
  bool btMode = false;
  double totalAngle = 0;
  Astar astar;
  double totalScanTime = 0;
  bool act = true;
  // RFID
  double absTagX =  std::stod(argv[11]); // m.
  double absTagY =  std::stod(argv[12]); // m.
  double freq = std::stod(argv[13]); // Hertzs
  double txtPower= std::stod(argv[14]); // dBs
  std::pair<int, int> relTagCoord;
  int rfidScan = 0;
  // RFID scan from the starting pose
//  relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
//  double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
//  double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
////  map.updateRFIDGrid(rxPower - MIN_SENSITIVITY, phase, target.getX(), target.getY());
////  ray.performRFIDSensingOperation( map, target.getX(),target.getY(), target.getOrientation() , target.getFOV(), target.getRange(), rxPower - MIN_SENSITIVITY, target.getScanAngles().first, target.getScanAngles().second );
//  rfidScan ++;

  do
  {
    if ( btMode == false )
    {
      long x = target.getX();
      long y = target.getY();
      int orientation = target.getOrientation();
      int range = target.getRange();
      double FOV = target.getFOV();
      string actualPose = function.getEncodedKey ( target,0 );
      map.setCurrentPose ( target );




      //-------------------------------------------------------------------

      string encoding = to_string ( target.getX() ) + to_string ( target.getY() );
      visitedCell.emplace ( encoding,0 );



      //cout << "-----------------------------------------------------------------"<<endl;
      //cout << "Round : " << count<< endl;
      //cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
      //cout << "Current position: x = " << y << " y = " << x << " orientation: " << orientation << std::endl;


      target.setScanAngles ( ray.getSensingTime ( map,x,y,orientation,FOV,range ) );
      // 	    cout << "MinPhi: " << target.getScanAngles().first << " MaxPhi: " << target.getScanAngles().second << endl;
      newSensedCells = sensedCells + ray.performSensingOperation ( map,x,y,orientation,FOV,range, target.getScanAngles().first, target.getScanAngles().second );
      // 	    cout << x << " " << y << endl;
      // 	    cout << newSensedCells << endl;
      double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      totalScanTime += calculateScanTime ( scanAngle*180/PI );
      relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
      double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
      cout << "rxPower: " << rxPower << endl;
      // debug ..................................
       double phi;
       double r;
      getSphericCoords(relTagCoord.first,relTagCoord.second, r, phi);
      cout << "Spheric coords are: (" << r << " m. ," << phi << " rad. ) \n";

      double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
      rfidScan ++;
      map.updatePathPlanningGrid ( x, y, range, rxPower - MIN_SENSITIVITY);
      ray.findCandidatePositions ( map,x,y,orientation,FOV,range );
      vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
      ray.emptyCandidatePositions();




      //cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells << endl;
      //--------------------------------------------------
      /* Push in the graph the initial position with different orientations
           */

      if ( act )
      {
        //NOTE; calculate path and turnings between actual position and goal
        /*string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
              travelledDistance = travelledDistance + astar.lenghtPath ( path );
              cout << astar.lenghtPath ( path ) << endl;
              numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
              numConfiguration++;
              */
        //totalAngle += scanAngle;
      }
      act = false;

      //cout << "Visited Positions: " << numConfiguration << std::endl;
      //cout << "distance " << travelledDistance << endl;

      if ( count == 0 )
      {
        string invertedPose = function.getEncodedKey ( invertedInitial,0 );
        string eastPose = function.getEncodedKey ( eastInitial,0 );
        string westPose = function.getEncodedKey ( westInitial,0 );
        list<Pose> empty ;
        std::pair<string,list<Pose>> pair1 = make_pair ( invertedPose,empty );
        std::pair<string,list<Pose>> pair2 = make_pair ( eastPose,empty );
        std::pair<string,list<Pose>> pair3 = make_pair ( westPose,empty );
        graph2.push_back ( pair1 );
        graph2.push_back ( pair2 );
        graph2.push_back ( pair3 );
      }

      if ( count != 0 && ( target.isEqual ( invertedInitial ) || target.isEqual ( eastInitial ) || target.isEqual ( westInitial ) ) )
      {
        if ( graph2.size() == 0 ) break;
        graph2.pop_back();
        actualPose = function.getEncodedKey ( target,0 );
        pushInitialPositions ( map, x, y,orientation, range,FOV, threshold, actualPose, &graph2 );
      }
      //------------------------------------------------------

      if ( candidatePosition.size() == 0 )
      {
        ray.findCandidatePositions2 ( map,x,y,orientation,FOV,range );
        candidatePosition = ray.getCandidatePositions();
        ray.emptyCandidatePositions();
      }

      if ( candidatePosition.size() == 0 )
      {

        //cout << "No other candidate position" << endl;
        //cout << "----- BACKTRACKING -----" << endl;

        if ( graph2.size() > 1 )
        {

          //get the last position in the graph and then remove it

          string targetString = graph2.at ( graph2.size()-1 ).first;
          graph2.pop_back();
          EvaluationRecords record;
          target = record.getPoseFromEncoding ( targetString );
          history.push_back ( function.getEncodedKey ( target,2 ) );
          //cout << "[BT]No significative position reachable. Come back to previous position" << endl;
          //cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
          count = count + 1;
          //cout << "Graph dimension : " << graph2.size() << endl;

        }
        else
        {
          cout << "Num configuration: " << numConfiguration << endl;
          cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;

          cout << "------------------ HISTORY -----------------" << endl;
          travelledDistance = 0;
          vector<string>::iterator it_history = history.begin();
          int history_counter = 0;

          list<Pose> tmp_history;
          EvaluationRecords record_history;
          for ( it_history; it_history!=prev(history.end(),1); it_history++)
          {

            if ((*it_history).back() == '1')
            {
              tmp_history.push_back(record_history.getPoseFromEncoding(*it_history));
            }
          }

          history_counter = tmp_history.size();
          list<Pose>::iterator p1_history = tmp_history.begin();

          for ( p1_history; p1_history!= prev ( tmp_history.end(),1 ); p1_history++ )
          {
            cout << function.getEncodedKey(*p1_history,1) << endl; // print cell in the tabulist
            list<Pose>::iterator p2_history = next ( p1_history,1 );
            string path = astar.pathFind ( ( *p2_history ).getX(), ( *p2_history ).getY(), ( *p1_history ).getX(), ( *p1_history ).getY(),map );
            travelledDistance = travelledDistance + astar.lenghtPath ( path );
            numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
            //cout << astar.lenghtPath ( path ) << endl;
          }
          cout << "History counter: " << history_counter << endl;
          cout << "Travelled distance calculated from the history: " << travelledDistance << endl;


          cout << "------------------ TABULIST -----------------" << endl;
          travelledDistance = 0;
          list<Pose>::iterator it = tabuList.begin();
          for ( it; it!= prev ( tabuList.end(),1 ); it++ )
          {
            cout << function.getEncodedKey(*it,1) << endl; // print cell in the tabulist
            std::list<Pose>::iterator it2 = next ( it,1 );
            string path = astar.pathFind ( ( *it2 ).getX(), ( *it2 ).getY(), ( *it ).getX(), ( *it ).getY(),map );
            travelledDistance = travelledDistance + astar.lenghtPath ( path );
            numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
            //cout << astar.lenghtPath ( path ) << endl;
          }
          cout << function.getEncodedKey(*prev(tabuList.end()),1) << endl;  // pribt last item of tabulist
          numConfiguration = tabuList.size();
          cout << "Travelled distance calculated from the tabulist: " << travelledDistance << endl;

          if ( imgresolution == 1.0 )
          {
            // full resolution is 0.5m*0.5m
            // Divide by two to obtain distance in meter
            travelledDistance = travelledDistance/2;
          }
          cout << "-----------------------------------------------------------------"<<endl;
          cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
          cout << "Total cell visited :" << numConfiguration <<endl;
          cout << "Total travelled distance (cells): " << travelledDistance << endl;
          cout << "I came back to the original position since i don't have any other candidate position"<< endl;
          cout << "Total exploration time: " << travelledDistance / 0.5 << endl;
          cout << "Total number of turning: " << numOfTurning << endl;
          cout << "Sum of scan angles (radians): " << totalAngle << endl;
          cout << "Total time for scanning: " << totalScanTime << endl;
          cout << "Total time for exploration: " << travelledDistance/0.5 + totalScanTime << "s, " << ( travelledDistance/0.5 + totalScanTime ) /60 << " m" << endl;
          cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
          cout << "-----------------------------------------------------------------"<<endl;
          exit ( 0 );
        }

        sensedCells = newSensedCells;

      }
      else
      {

        //cout << newSensedCells << endl;
        // need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
        list<Pose> frontiers;
        vector<pair<long,long> >::iterator it =candidatePosition.begin();
        for ( it; it != candidatePosition.end(); it++ )
        {
          Pose p1 = Pose ( ( *it ).first, ( *it ).second,0 ,range,FOV );
          Pose p2 = Pose ( ( *it ).first, ( *it ).second,45,range,FOV );
          Pose p3 = Pose ( ( *it ).first, ( *it ).second,90,range,FOV );
          Pose p4 = Pose ( ( *it ).first, ( *it ).second,135,range,FOV );
          Pose p5 = Pose ( ( *it ).first, ( *it ).second,180,range,FOV );
          Pose p6 = Pose ( ( *it ).first, ( *it ).second,225,range,FOV );
          Pose p7 = Pose ( ( *it ).first, ( *it ).second,270,range,FOV );
          Pose p8 = Pose ( ( *it ).first, ( *it ).second,315,range,FOV );
          frontiers.push_back ( p1 );
          frontiers.push_back(p2);
          frontiers.push_back ( p3 );
          frontiers.push_back(p4);
          frontiers.push_back ( p5 );
          frontiers.push_back(p6);
          frontiers.push_back ( p7 );
          frontiers.push_back(p8);

        }

        unexploredFrontiers = frontiers;

        //cout << "Graph dimension : " << graph2.size() << endl;
        //cout << "Candidate position: " << candidatePosition.size() << endl;
        //cout <<"Frontiers: "<<  frontiers.size() << endl;
        EvaluationRecords *record = function.evaluateFrontiers ( frontiers,map,threshold );
        //cout << "Record: " << record->size() << endl;
        //cout << "Evaluation Record obtained" << endl;
        nearCandidates = record->getFrontiers();

        if ( record->size() != 0 )
        {

          //NOTE: TAKE THIS BRANCH IF THERE ARE CANDIDATE POSITION

          //set the previous pose equal to the actual one(actually represented by target)
          previous = target;
          std::pair<Pose,double> result = function.selectNewPose ( record );
          target = result.first;
          if ( contains ( tabuList,target ) == false )
          {
            act = true;
            count = count + 1;
            // 			numConfiguration++;
            // 			cout << "ADDED" << x << " " << y << endl;
            history.push_back ( function.getEncodedKey ( target,1 ) );
            cout << function.getEncodedKey ( target,1 ) << endl;
            tabuList.push_back ( target );
            cleanPossibleDestination2 ( nearCandidates,target );
            std::pair<string,list<Pose>> pair = make_pair ( actualPose,nearCandidates );
            graph2.push_back ( pair );

            string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
            travelledDistance = travelledDistance + astar.lenghtPath(path);
            //cout << astar.lenghtPath ( path ) << endl;
            numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
            numConfiguration++;
            totalAngle += scanAngle;

//            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
//            double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
//            double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
//            rfidScan ++;
//            map.updateRFIDGrid(rxPower - MIN_SENSITIVITY, phase, target.getX(), target.getY());
//            ray.performRFIDSensingOperation( map, target.getX(),target.getY(), target.getOrientation() , target.getFOV(), target.getRange(), rxPower - MIN_SENSITIVITY, target.getScanAngles().first, target.getScanAngles().second );


            //cout << "Graph dimension : " << graph2.size() << endl;
          }
          else
          {
            if ( graph2.size() == 0 ) break;
            if ( graph2.at ( graph2.size()-1 ).second.size() != 0 )
            {
              //cout << "[BT1 - Tabulist]There are visible cells but the selected one is already explored!Come back to second best position from the previous position"<< endl;
              cleanPossibleDestination2 ( nearCandidates,target );
              record = function.evaluateFrontiers ( nearCandidates,map,threshold );
              if ( record->size() != 0 )
              {
                std::pair<Pose,double> result = function.selectNewPose ( record );
                target = result.first;
                tabuList.push_back ( target );
                history.push_back ( function.getEncodedKey ( target,1 ) );
                cout << function.getEncodedKey ( target,1 ) << endl;
                string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
                travelledDistance = travelledDistance + astar.lenghtPath(path);
                //cout << astar.lenghtPath ( path ) << endl;
                numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
                numConfiguration++;
                totalAngle += scanAngle;

//                relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
//                double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
//                double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
////                map.updateRFIDGrid(rxPower - MIN_SENSITIVITY, phase, target.getX(), target.getY());
////                ray.performRFIDSensingOperation( map, target.getX(),target.getY(), target.getOrientation() , target.getFOV(), target.getRange(), rxPower - MIN_SENSITIVITY, target.getScanAngles().first, target.getScanAngles().second );
//                rfidScan ++;


                count = count + 1;
                // 				numConfiguration++;
                // 				cout << "AAAAA" << endl;
                //cout << "Graph dimension : " << graph2.size() << endl;
                btMode = true;

              }
              else
              {
                if ( graph2.size() == 0 ) break;
                string targetString = graph2.at ( graph2.size()-1 ).first;
                graph2.pop_back();
                target = record->getPoseFromEncoding ( targetString );
              }
            }
            else
            {

              graph2.pop_back();
              string targetString = graph2.at ( graph2.size()-1 ).first;
              target = record->getPoseFromEncoding ( targetString );
              history.push_back ( function.getEncodedKey ( target,2 ) );
              //cout << "[BT2 - Tabulist]There are visible cells but the selected one is already explored!Come back to two position ago"<< endl;
              //cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
              count = count + 1;

              //cout << "Graph dimension : " << graph2.size() << endl;
            }

          }
        }
        else
        {

          //NOTE: TAKE THIS BRANCH IF THERE ARE NO CANDIDATE POSITIONS

          if ( graph2.size() == 0 ) break;


          //select as new target the previous one in the graph structure
          string targetString = graph2.at ( graph2.size()-1 ).first;
          graph2.pop_back();

          target = record->getPoseFromEncoding ( targetString );




          if ( !target.isEqual ( previous ) )
          {
            previous = target;
            //cout << "[BT3]There are no visible cells so come back to previous position in the graph structure" << endl;
            history.push_back ( function.getEncodedKey ( target,2 ) );
            //cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
            count = count + 1;

            //cout << "Graph dimension : " << graph2.size() << endl;

          }
          else
          {

            if ( graph2.size() == 0 )
            {
              //cout << "[BT4]No other possibilities to do backtracking on previous positions" << endl;
              break;
            }
            string targetString = graph2.at ( graph2.size()-1 ).first;
            graph2.pop_back();

            target = record->getPoseFromEncoding ( targetString );
            previous = target;
            //cout << "[BT5]There are no visible cells so come back to previous position" << endl;
            //cout << "[BT5]Cell already explored!Come back to previous position"<< endl;
            history.push_back ( function.getEncodedKey ( target,2 ) );
            //cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
            count = count + 1;

            //cout << "Graph dimension : " << graph2.size() << endl;
          }

        }



        //NOTE: not requested for testing purpose
        //usleep(microseconds);
        sensedCells = newSensedCells;
        frontiers.clear();
        candidatePosition.clear();
        delete record;
      }

    }
    else
    {

      //NOTE: BT MODE
      long x = target.getX();
      long y = target.getY();
      int orientation = target.getOrientation();
      int range = target.getRange();
      double FOV = target.getFOV();
      string actualPose = function.getEncodedKey ( target,0 );
      map.setCurrentPose ( target );
      //NOTE; calculate path and turnings between actual position and goal
      cout<< function.getEncodedKey ( target,1 ) << endl;
      string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
      travelledDistance = travelledDistance + astar.lenghtPath ( path );
      cout << "BT: " << astar.lenghtPath ( path ) << endl;
      numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );

      //-----------------------------------------------------------------
      string encoding = to_string ( target.getX() ) + to_string ( target.getY() );
      visitedCell.emplace ( encoding,0 );
      previous = target;

      // 	    cout << "-----------------------------------------------------------------"<<endl;
      //  	    cout << "|||| BT MODE ||||"<< endl;
      // 	    cout << "Round : " << count<< endl;
      // 	    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
      // 	    newSensedCells = sensedCells + ray.performSensingOperation(map,x,y,orientation,FOV,range, target.getScanAngles().first, target.getScanAngles().second);
      target.setScanAngles ( ray.getSensingTime ( map,x,y,orientation,FOV,range ) );
      // 	    cout << "MinPhi: " << target.getScanAngles().first << " MaxPhi: " << target.getScanAngles().second << endl;

      double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      totalAngle += scanAngle;
      totalScanTime += calculateScanTime ( scanAngle*180/PI );
      relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
      double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
      double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
      rfidScan ++;
      map.updatePathPlanningGrid ( x, y, range, rxPower - MIN_SENSITIVITY );
      //ray.findCandidatePositions(map,x,y,orientation,FOV,range);
      //vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
      //ray.emptyCandidatePositions();



      cleanPossibleDestination2 ( nearCandidates,target );
      // 	    cout << "nearCandidates dimensions before choosing : " << nearCandidates.size() << endl;
      EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates,map,threshold );
      // 	    cout << "Evaluation Record obtained with dimension :" << record->size() << endl;

      if ( record->size() != 0 )
      {

        //NOTE: TAKE THIS BRANCH IF THERE ARE CANDIDATE POSITION
        //set the previous pose equal to the actual one(actually represented by target)

        std::pair<Pose,double> result = function.selectNewPose ( record );
        target = result.first;
        if ( contains ( tabuList,target ) == false )
        {
          count = count + 1;
          // 		    numConfiguration++;
          history.push_back ( function.getEncodedKey ( target,1 ) );
          //cout << "Graph dimension : " << graph2.size() << endl;
          tabuList.push_back ( target );
          cout<< function.getEncodedKey ( target,1 ) << endl;
          string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
          travelledDistance = travelledDistance + astar.lenghtPath(path);
          cout << "BTL "<<astar.lenghtPath ( path ) << endl;
          numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
          numConfiguration++;
          totalAngle += scanAngle;

//          relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
//          double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
//          double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
////          map.updateRFIDGrid(rxPower - MIN_SENSITIVITY, phase, target.getX(), target.getY());
////          ray.performingOperation( map, target.getX(),target.getY(), target.getOrientation() , target.getFOV(), target.getRange(), rxPower - MIN_SENSITIVITY, target.getScanAngles().first, target.getScanAngles().second );
//          rfidScan ++;

          cleanPossibleDestination2 ( nearCandidates,target );
          std::pair<string,list<Pose>> pair = make_pair ( actualPose,nearCandidates );
          btMode = false;
          nearCandidates.clear();
          // 		    cout << "[BT-MODE4] Go back to previous positions in the graph" << endl;
        }
        else
        {
          if ( nearCandidates.size() != 0 )
          {
            // 			cout << "[BT-MODE1]Already visited" << endl;
            count = count + 1;
            cleanPossibleDestination2 ( nearCandidates,target );
            EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates,map,threshold );
            std::pair<Pose,double> result = function.selectNewPose ( record );
            target = result.first;
            // 			numConfiguration++;
            tabuList.push_back ( target );
            // history.push_back ( function.getEncodedKey ( target,1 ) );  // added recently but it was not here
            cout<< function.getEncodedKey ( target,1 ) << endl;
            string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
            cout << "BT :"<<astar.lenghtPath ( path ) << endl;

            travelledDistance = travelledDistance + astar.lenghtPath(path);
            numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
            numConfiguration++;
            totalAngle += scanAngle;

//            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
//            double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
//            double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
////            map.updateRFIDGrid(rxPower - MIN_SENSITIVITY, phase, target.getX(), target.getY());
////            ray.performRFIDSensingOperation( map, target.getX(),target.getY(), target.getOrientation() , target.getFOV(), target.getRange(), rxPower - MIN_SENSITIVITY, target.getScanAngles().first, target.getScanAngles().second );
//            rfidScan ++;

            // 			cout << "nearCandidates dimensions after choosing : " << nearCandidates.size() << endl;

          }
          else
          {
            // 			cout << "[BT-MODE2] Go back to previous positions in the graph" << endl;
            string targetString = graph2.at ( graph2.size()-1 ).first;
            graph2.pop_back();
            target = record->getPoseFromEncoding ( targetString );
            tabuList.push_back ( target );
            history.push_back ( function.getEncodedKey ( target,2 ) );
            //cout<< function.getEncodedKey ( target,1 ) << endl;
            string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
            cout <<"BT: "<< astar.lenghtPath ( path ) << endl;
            travelledDistance = travelledDistance + astar.lenghtPath(path);
            numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
            numConfiguration++;

            totalAngle += scanAngle;

            count++;
            // 			numConfiguration++;

            // 			cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
            btMode = false;
            nearCandidates.clear();
          }
        }
      }
      else
      {
        string targetString = graph2.at ( graph2.size()-1 ).first;
        graph2.pop_back();
        target = record->getPoseFromEncoding ( targetString );
        tabuList.push_back ( target );
        history.push_back ( function.getEncodedKey ( target,2 ) );
        string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(),map );
        cout <<"BT:"<< astar.lenghtPath ( path ) << endl;
        //cout<< function.getEncodedKey ( target,1 ) << endl;
        travelledDistance = travelledDistance + astar.lenghtPath(path);
        numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
        numConfiguration++;
        totalAngle += scanAngle;


        // 		cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
        count = count + 1;
        btMode = false;
        // 		cout << "[BT-MODE3] Go back to previous positions in the graph" << endl;
      }
      delete record;
      // 	    sensedCells = newSensedCells;

    }
  }
  while ( sensedCells < precision * totalFreeCells );
  map.drawVisitedCells ();
  map.printVisitedCells ( history );
  map.drawRFIDScan();

  cout << "Num configuration: " << numConfiguration << endl;
  cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;

  cout << "------------------ HISTORY -----------------" << endl;
  travelledDistance = 0;
  vector<string>::iterator it_history = history.begin();
  int history_counter = 0;

  list<Pose> tmp_history;
  EvaluationRecords record_history;
  for ( it_history; it_history!=prev(history.end(),1); it_history++)
  {

    if ((*it_history).back() == '1')
    {
      tmp_history.push_back(record_history.getPoseFromEncoding(*it_history));
    }
  }

  history_counter = tmp_history.size();
  list<Pose>::iterator p1_history = tmp_history.begin();

  for ( p1_history; p1_history!= prev ( tmp_history.end(),1 ); p1_history++ )
  {
    //cout << function.getEncodedKey(*p1_history,1) << endl; // print cell in the tabulist
    list<Pose>::iterator p2_history = next ( p1_history,1 );
    string path = astar.pathFind ( ( *p2_history ).getX(), ( *p2_history ).getY(), ( *p1_history ).getX(), ( *p1_history ).getY(),map );
    travelledDistance = travelledDistance + astar.lenghtPath ( path );
    numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
    //cout << astar.lenghtPath ( path ) << endl;
  }

  cout << "History counter: " << history_counter << endl;
  cout << "Travelled distance calculated from the history: " << travelledDistance << endl;


  cout << "------------------ TABULIST -----------------" << endl;
  travelledDistance = 0;
  list<Pose>::iterator it = tabuList.begin();
  for ( it; it!= prev ( tabuList.end(),1 ); it++ )
  {
    //cout << function.getEncodedKey(*it,1) << endl; // print cell in the tabulist
    std::list<Pose>::iterator it2 = next ( it,1 );
    string path = astar.pathFind ( ( *it2 ).getX(), ( *it2 ).getY(), ( *it ).getX(), ( *it ).getY(),map );
    travelledDistance = travelledDistance + astar.lenghtPath ( path );
    numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );
    //cout << astar.lenghtPath ( path ) << endl;
  }
  //cout << function.getEncodedKey(*prev(tabuList.end()),1) << endl;  // pribt last item of tabulist
  numConfiguration = tabuList.size();
  cout << "Tabulist counter: "<< numConfiguration << endl;
  cout << "Travelled distance calculated from the tabulist: " << travelledDistance << endl;

  if ( imgresolution == 1.0 )
  {
    // full resolution is 0.5m*0.5m
    // Divide by two to obtain distance in meter
    travelledDistance = travelledDistance/2;
  }


  if ( sensedCells >= precision * totalFreeCells )
  {
    cout << "-----------------------------------------------------------------"<<endl;
    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
    cout << "Total cell visited :" << numConfiguration <<endl;
    cout << "Total travelled distance (cells): " << travelledDistance << endl;
    cout << "Total travel time: " << travelledDistance / 0.5 << endl;
    cout << "Total number of turning: " << numOfTurning << endl;
    cout << "Sum of scan angles (radians): " << totalAngle << endl;
    cout << "Total time for scanning: " << totalScanTime << endl;
    cout << "Total time for exploration: " << travelledDistance/0.5 + totalScanTime << "s, " << ( travelledDistance/0.5 + totalScanTime ) /60 << " m" << endl;
    cout << "FINAL: MAP EXPLORED!" << endl;
    cout << "RFID scans performed: " << rfidScan << endl;
    cout << "-----------------------------------------------------------------"<<endl;


  }
  else
  {
    cout << "-----------------------------------------------------------------"<<endl;
    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
    cout << "I came back to the original position since i don't have any other candidate position"<< endl;
    cout << "Total cell visited :" << numConfiguration <<endl;
    cout << "Total travelled distance (cells): " << travelledDistance << endl;
    cout << "Total exploration time: " << travelledDistance / 0.5 << endl;
    cout << "Total number of turning: " << numOfTurning << endl;
    cout << "Sum of scan angles (radians): " << totalAngle << endl;
    cout << "Total time for scanning: " << totalScanTime << endl;
    cout << "Total time for exploration: " << travelledDistance/0.5 + totalScanTime << "s, " << ( travelledDistance/0.5 + totalScanTime ) /60 << " m" << endl;
    cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
    cout << "-----------------------------------------------------------------"<<endl;
  }
  /*
  double finalDistance = 0;
  cout << tabuList.size() << endl;
  int tmp = 0;
  list<Pose>::iterator it = tabuList.begin();
  for ( it; it!= prev ( tabuList.end(),1 ); it++ )
    {
      tmp++;
      std::list<Pose>::iterator it2 = next ( it,1 );
      string path = astar.pathFind ( ( *it2 ).getX(), ( *it2 ).getY(), ( *it ).getX(), ( *it ).getY(),map );
      finalDistance = finalDistance + astar.lenghtPath ( path );
      cout << astar.lenghtPath ( path ) << endl;
    }


  cout << "FINAL LENGTH from tabulist: " << finalDistance << endl;
  cout << tmp << endl;
  cout << history.size() << endl;
  vector<string>::iterator it3 = history.begin();
  for ( it3; it3 != history.end(); it3++ )
    {
      cout << *it3 << endl;
    }
    */
  auto endMCDM= chrono::high_resolution_clock::now();

  double totalTimeMCDM = chrono::duration<double,milli> ( endMCDM -startMCDM ).count();
  //     cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
  // 		totalTimeMCDM/60000 << " m "<< endl;

}

bool contains ( std::list<Pose>& list, Pose& p )
{
  bool result = false;
  MCDMFunction function;

  std::list<Pose>::iterator findIter = std::find ( list.begin(), list.end(), p );
  if ( findIter != list.end() )
  {
    //cout << "Found it: "<< function.getEncodedKey(p,0) <<endl;
    result = true;
  }

  return result;
}

void cleanPossibleDestination2 ( std::list< Pose >& possibleDestinations, Pose& p )
{
  MCDMFunction function;
  //cout<<"I remove "<< function.getEncodedKey(p,0) << endl;
  //cout << possibleDestinations->size() << endl;
  std::list<Pose>::iterator findIter = std::find ( possibleDestinations.begin(), possibleDestinations.end(), p );
  if ( findIter != possibleDestinations.end() )
  {
    //cout << function.getEncodedKey(*findIter,0) << endl;
    possibleDestinations.erase ( findIter );
  }
  else cout<< "not found" << endl;

}


void pushInitialPositions ( Map map, int x, int y, int orientation, int range, int FOV, double threshold, string actualPose, vector< pair< string, list< Pose > > >* graph2 )
{
  NewRay ray;
  MCDMFunction function;
  ray.findCandidatePositions ( map,x,y,orientation ,FOV,range );
  vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
  ray.emptyCandidatePositions();
  list<Pose> frontiers;
  vector<pair<long,long> >::iterator it =candidatePosition.begin();
  for ( it; it != candidatePosition.end(); it++ )
  {
    Pose p1 = Pose ( ( *it ).first, ( *it ).second,0 ,range,FOV );
    Pose p2 = Pose ( ( *it ).first, ( *it ).second,180,range,FOV );
    Pose p3 = Pose ( ( *it ).first, ( *it ).second,90,range,FOV );
    Pose p4 = Pose ( ( *it ).first, ( *it ).second,270,range,FOV );
    frontiers.push_back ( p1 );
    frontiers.push_back ( p2 );
    frontiers.push_back ( p3 );
    frontiers.push_back ( p4 );
  }
  EvaluationRecords *record = function.evaluateFrontiers ( frontiers,map,threshold );
  list<Pose>nearCandidates = record->getFrontiers();
  std::pair<string,list<Pose>> pair = make_pair ( actualPose,nearCandidates );
  graph2->push_back ( pair );
}

double calculateScanTime ( double scanAngle )
{
  return ( -7.2847174296449998e-006*scanAngle*scanAngle*scanAngle + 2.2131847908245512e-003*scanAngle*scanAngle + 1.5987873410233613e-001*scanAngle + 10 );
}

Pose createFromInitialPose ( int x, int y, int orientation, int variation, int range, int FOV )
{
  Pose tmp = Pose ( x,y, ( orientation + variation ) %360,FOV,range );
  return tmp;
}
