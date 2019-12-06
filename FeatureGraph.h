#ifndef FEATURE_H
#define FEATURE_H


#include <string>
#include <vector>
#include <map>
#include "GraphHelper.h"


using namespace std;

struct Neighbor_Node{
    Node* node;
    int weight;
    Neighbor_Node(Node* node, int weight): node(node), weight(weight){}
};

struct Graph_Node{
    Node* node;
    vector<Neighbor_Node*> neighbors;
    Graph_Node(Node* node): node(node){}
    int index;
};


class FeatureGraph {

public:
    // TODO add necessary private fields


    // Constructor
    // 
    // N: The number of nodes
    // d: the sizee of the skill vectors of all nodes
    // nodes: vector of nodes to insert
    // edges: vector of edges to insert
    FeatureGraph(int N, int d, vector<Node> nodes, vector<Edge> edges);


    //Insert node with given ID and feature vectors
    void insert(Node node);
    void erase(int node_id);
    void erase(int id1, int id2);

    // insert given edge
    // May assume nodes in edge are already present
    void insert(Edge edge);

    vector<Graph_Node> getGraph();

private:
    vector<Graph_Node> graph;
};  

#endif
