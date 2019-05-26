#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
	start_x *= 0.01;
  	start_y *= 0.01;
  	end_x *= 0.01;	
  	end_y *= 0.01;
  	start_node = &m_Model.FindClosestNode(start_x, start_y);
  	end_node = &m_Model.FindClosestNode(end_x, end_y);
  	//end_node = &m_Model.FindClosestNode(end_x, end_y);
}

//construct the final path given the 
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node) {
	std::vector<RouteModel::Node> path_found;
  	RouteModel::Node parent;
  	distance = 0.0;
  	while(current_node->parent != nullptr) {
      	path_found.push_back(*current_node);
      	parent = *(current_node->parent);
      	distance += current_node->distance(parent);
    	current_node = current_node->parent;
    }
  	path_found.push_back(*current_node);
  	distance *= m_Model.MetricScale();
  	return path_found;
}

void RoutePlanner::AStarSearch() {
	start_node->visited = true;
	open_list.push_back(start_node);
	RouteModel::Node* current_node = nullptr;
	while(open_list.size() > 0) {
		current_node = NextNode();
		if (current_node->distance(*end_node) == 0) {
			m_Model.path = ConstructFinalPath(current_node);
			return;
		}
		else{
			AddNeighbors(current_node);
		}
	}
}

float RoutePlanner::CalculateHValue(RouteModel::Node* node) {
	return node->distance(*end_node);
}

RouteModel::Node* RoutePlanner::NextNode() {
	RouteModel::Node* node_ptr_copy;
	std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* node1, RouteModel::Node* node2) {
		return (node1->g_value + node1->h_value) < (node2->g_value + node2->h_value); 
	});
	node_ptr_copy = open_list.front();
	open_list.erase(open_list.begin());
	return node_ptr_copy;
}

void RoutePlanner::AddNeighbors(RouteModel::Node* node) {
	node->FindNeighbors();
	for(auto neighbor : node->neighbors) {
		neighbor->parent = node;
		neighbor->g_value = node->g_value + node->distance(*neighbor);
		neighbor->h_value = CalculateHValue(neighbor);
		open_list.push_back(neighbor);
		neighbor->visited = true;
	}
}