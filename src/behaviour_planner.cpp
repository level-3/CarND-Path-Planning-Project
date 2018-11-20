
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "road.h"
#include "vehicle.h"
using namespace std;

//impacts default behavior for most states
int SPEED_LIMIT = 50;
//all traffic in lane (besides ego) follow these speeds
vector<int> LANE_SPEEDS = {50,50,50}; 

//Number of available "cells" which should have traffic
double TRAFFIC_DENSITY   = 0.0;

// At each timestep, ego can set acceleration to value between 
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

// s value and lane number of goal.
//vector<int> GOAL = {300, 0};

// These affect the visualization
//int FRAMES_PER_SECOND = 50;
int AMOUNT_OF_ROAD_VISIBLE = 100;


int behaviour_planner(vector< vector<int> > GRID, vector<int> GOAL, int SPEED_LIMIT, vector<int> LANE_SPEEDS ) {
 
	Road road = Road(SPEED_LIMIT,  GRID, LANE_SPEEDS);

	road.update_width = AMOUNT_OF_ROAD_VISIBLE;

    road.populate_traffic();

	int goal_s = GOAL[0];
	int goal_lane = GOAL[1];

	//configuration data: speed limit, num_lanes, goal_s, goal_lane, max_acceleration

	int num_lanes = LANE_SPEEDS.size();
	vector<int> ego_config = {SPEED_LIMIT,num_lanes,goal_s,goal_lane,MAX_ACCEL};
	 
	road.add_ego(2,0, ego_config);

    

	int timestep = 0;
	
	while (road.get_ego().s <= GOAL[0]) {
		timestep++;


		road.advance();
		road.display();
		//time.sleep(float(1.0) / FRAMES_PER_SECOND);
	}
	Vehicle ego = road.get_ego();
	
    if (ego.lane == GOAL[1])
	{
		cout << "You got to the goal in " << timestep << " seconds!" << endl;
		if(timestep > 35)
	    {
	        cout << "But it took too long to reach the goal. Go faster!" << endl;
	    }
	}
	else
	{
		cout << "You missed the goal. You are in lane " << ego.lane << " instead of " << GOAL[1] << "." << endl;
	}

	return 0;
}