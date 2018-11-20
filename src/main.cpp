#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <cstdlib>
#include "hybrid_breadth_first.cpp"
using namespace std;
#include "JMT.cpp"

/*
#include "road.h"
#include "road.cpp"
#include "vehicle.h"
#include "vehicle.cpp"
*/

#include <iomanip>

//#include "python2.7/Python.h"
//#include "matplotlibcpp.h"

//#include "madplotlib.h"


//namespace plt = matplotlibcpp;
//using plt = matplotlibcpp::plt;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}



//-------------------------------------------------
            //impacts default behavior for most states
            int SPEED_LIMIT = 1;
            //all traffic in lane (besides ego) follow these speeds
            vector<int> LANE_SPEEDS = {1,1,1}; 
            // At each timestep, ego can set acceleration to value between 
            // -MAX_ACCEL and MAX_ACCEL
            int MAX_ACCEL = .1;

            // s value and lane number of goal.
            //vector<int> GOAL = {300, 0};
            int AMOUNT_OF_ROAD_VISIBLE = 40;


            //Road road = Road(SPEED_LIMIT,  GRID, LANE_SPEEDS);


int behaviour_planner(double car_s, int lane, vector<vector<double>> vehicle_list , vector< vector<int> > GRID, vector<int> GOAL, int SPEED_LIMIT, vector<int> LANE_SPEEDS ) {
 
	Road road = Road(SPEED_LIMIT,  GRID, LANE_SPEEDS);

	road.update_width = AMOUNT_OF_ROAD_VISIBLE;

  road.populate_traffic(vehicle_list);

  cout << "GOAL " << GOAL[0] << "," << GOAL[1] << endl;

  GOAL = {(int)car_s + 30, lane};

	int goal_s = GOAL[0];
	int goal_lane = GOAL[1];

  cout << "GOAL1 " << GOAL[0] << "," << GOAL[1] << endl;
  	//configuration data: speed limit, num_lanes, goal_s, goal_lane, max_acceleration

	int num_lanes = LANE_SPEEDS.size();

	vector<int> ego_config = {SPEED_LIMIT, num_lanes, goal_s, goal_lane, MAX_ACCEL};
	 
	road.add_ego(lane, car_s, ego_config);

  
	int timestep = 1;

	while (road.get_ego().s <= GOAL[0]) 
    {
    road.advance();
    		
    //time.sleep(float(1.0) / FRAMES_PER_SECOND);
    }
    road.display();
  
  Vehicle ego = road.get_ego();
	
  cout << timestep << ":" << ego.state  << endl;
  
    if (ego.lane == GOAL[1])
      {
        cout << "You got to the goal in " << timestep << " seconds!" << endl;
          if(timestep > 35)
            {
                cout << "But it took too long to reach the goal. Go faster!" << endl;
            }
      timestep = 1;
      }
      else
      {
        cout << "You missed the goal. You are in lane " << ego.lane << " instead of " << GOAL[1] << "." << endl;
      }


      timestep++;

  
	return 0;
}


//________________________________________________


int X = 1;
int _ = 0;

double SPEED = 0.1;
double LENGTH = 1.0;

vector<HBF::maze_s> hbfsearch(vector< vector<int> > GRID, int lane, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
  
  int X = 1;
  int _ = 0;

  double SPEED = 1.0;
  double LENGTH = 0.5;

  vector< vector<int> > MAZE = {
      {_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,},
      {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,},
      {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,},
      {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,},
      {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,},
      {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,},
      {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,},
      {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,},
      {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,},
      {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,},
      {_,X,X,_,_,X,X,X,_,_,X,X,X,X,X,X,},
      {_,_,X,X,X,X,X,_,_,X,X,X,X,X,X,X,},
      {_,_,_,X,X,X,X,X,X,X,X,X,X,X,X,X,},
      {_,_,_,_,_,_,X,X,X,X,X,X,X,X,X,X,},
      {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
      {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,},
  };

  vector< vector<int> > MAZE_6x6 = {
      {_,_,_,X,X,X,},
      {_,_,_,X,X,X,},
      {_,_,_,X,X,X,},
      {_,_,_,X,X,X,},
      {_,_,_,X,X,X,},
      {_,_,_,X,X,X,},
  };

  vector< vector<int> > MAZE_3x3 = {
      {_,_,_,},
      {_,_,_,},
      {_,_,_,},
  };

  //vector< vector<int> > 
  //GRID = MAZE_3x3;
  /**/

  vector<double> START = {0.0,(double)lane,0.0};
  vector<int> GOAL = {(int)GRID.size()-1, lane};
  GRID[0][lane] = 8;

  cout << "Finding path through grid: cell " << GOAL[0] << " lane " << GOAL[1] << endl;
   
  HBF hbf = HBF();

  HBF::maze_path get_path = hbf.search(GRID,START,GOAL);

  return hbf.reconstruct_path(get_path.came_from, START, get_path.final);


}





//________________________________________________




int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  vector< vector<int> > MAZE_3x10 = {
  {_,_,_},
  {_,_,_},
  {_,_,_},
  {_,_,_},
  {_,_,_},
  {_,_,_},
  {_,_,_},
  {_,_,_},
  {_,_,_},
  {_,_,_},
  };

  
vector< vector<int> > MAZE_6x6 = {
    {_,_,_,X,X,X,},
    {_,_,_,X,X,X,},
    {_,_,_,X,X,X,},
    {_,_,_,X,X,X,},
    {_,_,_,X,X,X,},
    {_,_,_,X,X,X,},
};

  vector<vector<int>> GRID = MAZE_6x6;
  vector<int> GOAL = {(int)0, (int)0};

  int lane = 1;
  double ref_vel = 0.0;
  int timestep = 0;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane , &ref_vel, &GRID, &GOAL, &timestep](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


            system("clear");
            cout << "ego" << endl << "\tx:y " << car_x << ":" << car_y  << "\ts:d " << car_s << ":" << car_d  << endl;
            //cout << "yaw:" << car_yaw << "\tspeed:" << car_speed << endl;

            auto XY = getXY(car_s , car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);            
            //cout << "XY\t" << XY[0] << ":" << XY[1] << endl;

            auto FN = getFrenet(car_x , car_y, car_yaw, map_waypoints_x, map_waypoints_y);
            //cout << "\tfrenet " << FN[0] << ":" << FN[1] << endl;

            int closest_waypoint_id = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);
            int next_waypoint_id = NextWaypoint(car_x, car_y,car_yaw, map_waypoints_x, map_waypoints_y);

            cout << "Closest : " << closest_waypoint_id << "\tNext : " << next_waypoint_id << endl; 


            auto CheckpointXY = getXY(map_waypoints_x[closest_waypoint_id] , map_waypoints_y[closest_waypoint_id], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            cout << next_waypoint_id << ": X:Y: " << CheckpointXY[0] << ":" << CheckpointXY[1] << endl; 

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // Provided previous path point size.
            int path_size = previous_path_x.size();

            
            if (path_size > 0) {car_s = end_path_s;}


            bool car_front = false;
            bool car_left = false;
            bool car_right = false;

         

            GRID = {
                {_,_,_,X,X,X,},
                {_,_,_,X,X,X,},
                {_,_,_,X,X,X,},
                {_,_,_,X,X,X,},
                {_,_,_,X,X,X,},
                {_,_,_,X,X,X,},
            };

            vector<vector<double>> vehicle_list ;
            for ( int i = 0; i < sensor_fusion.size(); i++ ) 
            {   
                //int id = sensor_fusion[i][0];
                double x = sensor_fusion[i][1];
                double y = sensor_fusion[i][2];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double s = sensor_fusion[i][5];
                double d = sensor_fusion[i][6];
                vector<double> entry = {x, y, vx, vy, s, d };
                vehicle_list.push_back(entry);
            }

              cout <<  "x " << "\t" 
                   <<  "y " << "\t" 
                   << "vx " << "\t" 
                   << "vy " << "\t" 
                   <<  "s " << "\t" 
                   <<  "d " << endl ;


            cout << fixed << setprecision(0);
            for ( int i = 0; i < vehicle_list.size(); i++ ) 
            {
              cout <<  vehicle_list[i][0] << "\t" 
                   <<  vehicle_list[i][1] << "\t" 
                   <<  vehicle_list[i][2] << "\t" 
                   <<  vehicle_list[i][3] << "\t" 
                   <<  vehicle_list[i][4] << "\t" 
                   <<  vehicle_list[i][5] << endl ;
            }


 
            //GRID[0][lane] = 8;
            //cout << GRID.size() << endl;

            int grid_size =10;

            for ( int i = 0; i < sensor_fusion.size(); i++ ) 
            {

                int car_lane = 1;

                //  car speed
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                float d = sensor_fusion[i][6];


                     if ( d > 0 && d < 4 ) {car_lane = 0;} 
                else if ( d > 4 && d < 8 ) {car_lane = 1;} 
                else if ( d > 8 && d < 12 ) {car_lane = 2;}
                if (car_lane < 0) {continue;}
/*
                car_lane = floor ( (d - 2) / 4);
*/
                     if ( car_lane == lane ) {car_front |= check_car_s > car_s && check_car_s - car_s < 30;}// Car is in the same lane
                else if ( car_lane - lane == -1 ) {car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;} // Car is on the left
                else if ( car_lane - lane == 1 ) {car_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;}// Car is on the right

                
                int grid_idx = floor(( check_car_s-car_s+ grid_size ) / grid_size)  ;

                if (grid_idx < 5 && grid_idx >= -1) 
                {
                  GRID[grid_idx+1][car_lane] = 1 ;

                  if (car_lane != lane)
                  {
                      //cout << "\t";
                  }
                  //cout << car_lane << ":" << grid_idx << "\ts: " << check_car_s - car_s  << endl;
                }

          

                
            }




//+++++++++++++++++++++++++<< BEHAVIOUR PLANNER >>+++++++++++++++++++++++++++++++++++++=
         

            //behaviour_planner(car_s, lane , vehicle_list, GRID, GOAL, SPEED_LIMIT, LANE_SPEEDS);


            // Behavior :

            double speed_diff = 0;
            const double MAX_SPEED = 49.5;
            const double MAX_ACC = .224;


            switch ((int)car_front)
            {
              case 0: cout << "No Car in Front" << endl;
                      if ( lane != 1 ) { // car isnt in the center
                        if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) ) {
                          lane = 1; // Back to center
                        }
                      }
                      if ( ref_vel < MAX_SPEED ) {speed_diff += MAX_ACC;}
                      break;
              case 1: cout << "Car is in Front" << endl;                  
                      if ( !car_left && lane > 0 ) {lane--;} 
                        // change lanes if there is no car on the left and there is also a left lane
                      else if ( !car_right && lane != 2 ){lane++;} 
                        // change lanes if there is no car on the right and there is also a right lane
                      else {speed_diff -= MAX_ACC;}   
                      break;
            }
            

//+++++++++++++++++++++++++<< ================= >>+++++++++++++++++++++++++++++++++++++=   

///////////////////////////////////////////HYBRID SEARCH////////////////////////////////////////////////////////////////


            vector<HBF::maze_s> show_path = hbfsearch(GRID, lane, map_waypoints_x, map_waypoints_y);
              
              //cout << "show path from start to finish" << endl;

              for(int i = show_path.size()-1; i >= 0; i--)
              {
                  cout << fixed << setprecision(0) ; 
                  HBF::maze_s step = show_path[i];
                  //cout << step.g << " # " ;
                  //cout << "x " << step.x << " ";
                  //cout << "\ty " << floor(step.y) ;
                  //cout << "\ttheta " << step.theta ;
                  
                  //vector<double> coord_frenet =  getFrenet(step.x, step.y, step.theta, map_waypoints_x, map_waypoints_y);
                  
                  //cout << "\ts " << step.x * grid_size + car_s << " ";
                  //cout << "\td " << (floor(step.y) * 4 ) + 2 << endl; 

                  GRID[  floor(step.x)  ][ floor(step.y)] = 8;

              }
              
              for(int i = GRID.size()-1 ; i >= 0; i--)
              {
                cout << i << " " << GRID[i][0];

                for(int j = 1; j < GRID[0].size()-3; j++)
                {
                  cout << " " << GRID[i][j];
                }
                cout << endl<<endl;
              }

              cout << "  0 1 2"<<endl;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ====================================<< Jerk Minimising Trajectory >>===================================================



            //cout << next_y_vals[0] << " " << next_y_vals[next_y_vals.size()] << endl;


            int steps = next_y_vals.size();
            for( int i = 0; i <= steps ; i++ ) 
            { 
              float T = 1.0;
              float timestep = (double)i/(double)steps;

              //cout << fixed << std::setprecision(0);
              //cout << next_x_vals[i] << ":" << next_y_vals[i] << "\t";

              vector<double> result = JMT({car_d, 0 , 0.0}, { 4* (double)lane + 2 , 0, 0.0}, T );

              double s_val = result[0] + result[1] * timestep + result[2] * pow(timestep,2) + result[3] * pow(timestep,3) + result[4] * pow(timestep,4) + result[5] * pow(timestep,5);

              //next_y_vals[i] = s_val;

              cout << fixed << std::setprecision(2);
              //cout << i << "\t" << timestep  <<  "\t" << s_val << "\t"  << endl ;
            }        

            //vector<int> GOAL = {(int)0, (int)lane};

            
// ====================================<< -------------------------- >>===================================================            
  




          	vector<double> waypoint_x;
            vector<double> waypoint_y;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

        
            if ( path_size < 2  ) {

              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              
              waypoint_x.push_back(prev_car_x);
              waypoint_x.push_back(car_x);
              
              waypoint_y.push_back(prev_car_y);
              waypoint_y.push_back(car_y);

            } 
            else 
            {
                // Use the last two points.
                ref_x = previous_path_x[path_size - 1];
                ref_y = previous_path_y[path_size - 1];

                double ref_x_prev = previous_path_x[path_size - 2];
                double ref_y_prev = previous_path_y[path_size - 2];

                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                waypoint_x.push_back(ref_x_prev);
                waypoint_x.push_back(ref_x);

                waypoint_y.push_back(ref_y_prev);
                waypoint_y.push_back(ref_y);
              }




            double distance = 35.0 ;
            double d_add = 2 + 4*lane ;

            // Setting up target points in the future.
            vector<double> next_wp0 = getXY(car_s + distance  , d_add , map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + distance*2, d_add , map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + distance*3, d_add , map_waypoints_s, map_waypoints_x, map_waypoints_y);


            waypoint_x.push_back( next_wp0[0] );
            waypoint_x.push_back( next_wp1[0] );
            waypoint_x.push_back( next_wp2[0] );

            waypoint_y.push_back( next_wp0[1] );
            waypoint_y.push_back( next_wp1[1] );
            waypoint_y.push_back( next_wp2[1] );




            // Making coordinates to local car coordinates.
            for ( int i = 0; i < waypoint_x.size(); i++ ) 
            {
              double shift_x = waypoint_x[i] - ref_x;
              double shift_y = waypoint_y[i] - ref_y;

              waypoint_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              waypoint_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // create the spline
            tk::spline s;
            s.set_points(waypoint_x,waypoint_y);


            for ( int i = 0; i < path_size; i++ ) 
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }


            // calculate distance ahead
            double target_x = distance;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;


            for( int i = 1; i < 50 - path_size; i++ ) 
            {
              ref_vel += speed_diff;

              if ( ref_vel > MAX_SPEED ) {ref_vel = MAX_SPEED;} 
              else if ( ref_vel < MAX_ACC ) {ref_vel = MAX_ACC;}

              double N = target_dist/(0.02*ref_vel/2.24);

              double x_point = x_add_on + target_x/N;
              
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }


          


          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}