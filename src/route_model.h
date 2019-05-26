#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
  	
    class Node : public Model::Node {
      public:
      	void FindNeighbors();
        // Add public Node variables and methods here.
        float distance(Node node) const {
        	return std::sqrt(std::pow(node.x - x, 2)+std::pow(node.y - y, 2));
        }
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      	Node* parent = nullptr;
      	float h_value = std::numeric_limits<float>::max();
      	float g_value = 0.0;
      	bool visited = false;
      	std::vector<Node*> neighbors;
      private:
        // Add private Node variables and methods here.
      	Node* FindNeighbor(std::vector<int> node_indices);
        int index;
        RouteModel * parent_model = nullptr;
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
    auto &SNodes(){return m_Nodes;} //has to be reference, because m_Nodes is a vecto 
  	auto &GetNodeToRoadMap() {return node_to_road;}
  	Node &FindClosestNode(float x, float y);
  	
  private:
  	void CreateNodeToRoadHashmap(); //
  	std::vector<Node> m_Nodes; 
  	
  	std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;
    // Add private RouteModel variables and methods here.
	
};

#endif
