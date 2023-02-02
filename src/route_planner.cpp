#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    // Store the nodes the RoutePlanner's start_node and end_node attributes.
	
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculate H Distance Value.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	float dist = node->distance(*end_node);
  	return dist;
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
  
  	for (RouteModel::Node *neighbor_node: current_node->neighbors){
    	neighbor_node->parent = current_node;
      	neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value + current_node->distance(*neighbor_node);
      	
      	open_list.emplace_back(neighbor_node);
      	neighbor_node->visited = true;
      	
    }
}


// Sort the open list 
bool Compare(const RouteModel::Node *a, const RouteModel::Node *b){
  float f_value1 = a->h_value + a->g_value;
  float f_value2 = b->h_value + b->g_value;
  return (f_value1 > f_value2);
}

// Return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
	std::sort(open_list.begin(), open_list.end(), Compare);
  	RouteModel::Node* low_sum_node = open_list.back();
  	open_list.pop_back();
  	return low_sum_node;
}


// Return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
      
	while (current_node != start_node){
  	  path_found.emplace_back(*current_node);
      distance += current_node->distance(*(current_node->parent));
      current_node = current_node->parent;
    }
   
    path_found.emplace_back(*current_node);
  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  	std::reverse(path_found.begin(), path_found.end());
    return path_found;
}

// A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

  current_node = start_node;
  current_node->visited = true;
  open_list.emplace_back(current_node);
  
  while ((open_list).size() > 0){
  	current_node = NextNode();
    if (current_node == end_node){
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    AddNeighbors(current_node);
  }
}





