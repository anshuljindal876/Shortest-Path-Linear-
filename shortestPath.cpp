/*
	shortestPath.cpp - Library for shortest path algorithm valid for linear open path with unit node distances.
	Created by Anshul Jindal, July 04, 2018.
	Only for VSSUT Robotics Club Members.
*/

#include "Arduino.h"
#include "shortestPath.h"

//simplified versions of for loops
#define F(chk, st, en) for(int chk = st; chk < en; chk++)
#define _F(chk, st, en) for(int chk = st; chk > en; chk--)


int shortestPath::nearest_newNode(int ref, bool typ)
/*
    gets context from the connection matrix and checks whether the next/prev node is connected to ref node.
    also checks if that node is visited or not and limits checking between nodes (0 to n)
    returns value of the nearest, connected unvisited node for 'ref node'.
    if no such node exists, returns and error value
*/
{
  if (typ) // if motion_dir is true, first check next node then previous.
  {
    F(incr_1, 1, _numNodes)  // increment difference between nodes checked
    {
      if (ref + incr_1 < _numNodes && _connection[ref][ref + incr_1] == 1 && !visited_nodes[ref + incr_1]) return ref + incr_1;

      if (ref - incr_1 >= 0 && _connection[ref][ref - incr_1] == 1 && !visited_nodes[ref - incr_1]) return ref - incr_1;
    }
  }
  else   // if motion_dir is false, first check prev node then next.
  {
    F(incr_1, 1, _numNodes)  // increment difference between nodes checked
    {
      if (ref - incr_1 >= 0 && _connection[ref][ref - incr_1] == 1 && !visited_nodes[ref - incr_1])  return ref - incr_1;
        
      if (ref + incr_1 < _numNodes && _connection[ref][ref + incr_1] == 1 && !visited_nodes[ref + incr_1])  return ref + incr_1;
    }
  }
  return err_val;
}

void shortestPath::reset()
{
  dist = 0, from_realT  = 0, to_realT = 0;
  prev_dist = 0;
  prev_from = 0;
  motion_dir = true;
}

void shortestPath::calc_path(int numNodes, int connection[20][20], int starting_node, int target_node)
{
  _numNodes = numNodes;
  err_val = _numNodes + 1;

  F(incr_2, 0, 20) 
  {
    visited_nodes[incr_2] = false;
    node_path[incr_2] = err_val;
    F(incr_3, 0, 20)
    {
      _connection[incr_2][incr_3] = connection[incr_2][incr_3];
    }
  }

  if (starting_node > target_node) motion_dir = !motion_dir;

  if (starting_node != target_node)
  {
    visited_nodes[starting_node] = true;      		 // starting_nodeing point is visited
    node_path[0] = starting_node;             		 // starting_node path from given starting_nodeing node
    from_realT  = starting_node;              		 // from_realT  is the real-time 'FROM' variable.

check_path:                           		 // goto statement to avoid complicated for-loops
    to_realT = nearest_newNode(from_realT, motion_dir);     // to_realT is the real-time 'TO' variable; gets nearest, non visited node value
    if (to_realT != err_val)                 // when a 'next node' is available.
    {
      dist += 1;                      		 // add distance.
      visited_nodes[to_realT] = true;
      node_path[dist] = to_realT;            // add node to path followed
      if (to_realT == target_node)                // if target_node is acheieved
      {
        return;
      }
      else                            // if target_node is not achieved
      {
        prev_from = from_realT ;
        prev_dist = dist;             // temp. storing distance value
        from_realT  = to_realT;
        goto check_path;              // to search for path ahead
      }
    }
    else  							  // if no 'next node' is available, revert back to previous state
    {
      node_path[dist--] = 10;		  // to 'unvisit' node and decrease distance travelled
      from_realT  = node_path[dist];
      goto check_path;
    }
  }
  else                                // when target_node is same as starting_node node   
  {
    dist = 0;
    node_path[0] = starting_node;
    visited_nodes[starting_node] = true;
  }
}