#include <iostream>
//#include "road.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */
Road::Road(int speed_limit, vector< vector<int> > GRID, vector<int> lane_speeds) {

    this->num_lanes = lane_speeds.size();
    this->lane_speeds = lane_speeds;
    this->speed_limit = speed_limit;
    this->density = 0;
    this->GRID = GRID;
    this->camera_center = this->update_width/2;

}

Road::~Road() {}

Vehicle Road::get_ego() {
	
	return this->vehicles.find(this->ego_key)->second;
}



void Road::populate_traffic( vector<vector<double>> vehicle_list) {
	
	int start_s = max(this->camera_center - (this->update_width/2), 0);
	
    for (int v = 0; v < vehicle_list.size(); v++)
	{

                int l = (vehicle_list[v][5] - 2 ) / this->lane_width; // floor(vehicle_list[v][5] / num_lanes);
                float s =  vehicle_list[v][4];
                int lane_speed = this->lane_speeds[l];

				Vehicle vehicle = Vehicle() ; //l,s,lane_speed,0,"CS");
                vehicle.lane = l ;
                vehicle.s = s ;
                vehicle.v = v ;
                vehicle.a = lane_speed ; 
				vehicle.state = "CS";
				this->vehicles_added += 1;
				this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
				//vehicle_just_added = true;
        
    }



    for (int l = 0; l < this->num_lanes; l++)
	{
		int lane_speed = this->lane_speeds[l];
		bool vehicle_just_added = false;

	}
	
}

void Road::advance() {
	
	map<int ,vector<Vehicle> > predictions;

	map<int, Vehicle>::iterator it = this->vehicles.begin();

    while(it != this->vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        it++;
    }
    
	it = this->vehicles.begin();
	while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        if(v_id == ego_key)
        {   
        	vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
        	it->second.realize_next_state(trajectory);
        }
        else {
            it->second.increment(0.02);
        }
        it++;
    }
    
}


void Road::add_ego(int lane_num, int s, vector<int> config_data) {
	
	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if(v.lane == lane_num && v.s == s)
        {
        	this->vehicles.erase(v_id);
        }
        it++;
    }
    Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);
    ego.configure(config_data);
    ego.state = "KL";
    this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
    
}

void Road::display() {

    Vehicle ego = this->vehicles.find(this->ego_key)->second;
    int s = ego.s;
    string state = ego.state;

    this->camera_center = max(s, this->update_width/2);
    int s_min = max(this->camera_center - this->update_width/2, 0);
    int s_max = s_min + this->update_width;

    vector<vector<string> > road;

    for(int i = 0; i < this->update_width; i++)
    {
        vector<string> road_lane;
        for(int ln = 0; ln < this->num_lanes; ln++)
        {
            road_lane.push_back("     ");
        }
        road.push_back(road_lane);

    }

    map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {

        int v_id = it->first;
        Vehicle v = it->second;

        if(s_min <= v.s && v.s < s_max)
        {
            string marker = "";
            if(v_id == this->ego_key)
            {
                marker = this->ego_rep;
            }
            else
            {
                
                stringstream oss;
                stringstream buffer;
                buffer << " ";
                oss << v_id;
                for(int buffer_i = oss.str().length(); buffer_i < 3; buffer_i++)
                {
                    buffer << "0";
                
                }
                buffer << oss.str() << " ";
                marker = buffer.str();
            }
            road[int(v.s - s_min)][int(v.lane)] = marker;
        }
        it++;
    }
    ostringstream oss;
    oss << "+Meters ======================+ step: "  << endl;
    int i = s_min;
    for(int lj = 0; lj < road.size(); lj++)
    {
        if(i%20 ==0)
        {
            stringstream buffer;
            stringstream dis;
            dis << i;
            for(int buffer_i = dis.str().length(); buffer_i < 3; buffer_i++)
            {
                 buffer << "0";
            }
            
            oss << buffer.str() << dis.str() << " - ";
        }
        else
        {
            oss << "      ";
        }          
        i++;
        for(int li = 0; li < road[0].size(); li++)
        {
            oss << "|" << road[lj][li];
        }
        oss << "|";
        oss << "\n";
    }
    
    cout << oss.str();

}


