#ifndef BEHAVIOUR_PLANNER_H
#define BEHAVIOUR_PLANNER_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>


using namespace std;

class BehaviourPlanner {
public:

  /**
  * Constructor
  */
  BehaviourPlanner();
  BehaviourPlanner(Road road);

  /**
  * Destructor
  */
  virtual ~BehaviourPlanner();

    int behaviour_planner(vector< vector<int> > GRID, vector<int> GOAL, int SPEED_LIMIT, vector<int> LANE_SPEEDS ) ;
 

};

#endif