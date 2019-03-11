/*
	shortestPath.h - Header file for shortest path algorithm valid for linear open path with unit node distances.
	Created by Anshul Jindal, July 04, 2018.
	Only for VSSUT Robotics Club Members.
*/

#ifndef shortestPath_h
	#define shortestPath_h

#include "Arduino.h"

class shortestPath
{
	private:
		int prev_from;                       // for temporary storage of previous value of j
		int prev_dist = 0;                // for temporary storage of previous value of dist
		int err_val;
		bool motion_dir = true;
		int from_realT = 0, to_realT = 0;
		int _numNodes;
		int _connection[20][20];
		int nearest_newNode(int ref, bool typ);
		bool visited_nodes[20];
	public:
		int dist = 0;
		int node_path[20];
		void reset();
		void calc_path(int numNodes, int connection[20][20], int starting_node, int target_node);
};

#endif