#ifndef ANALYZER_H
#define ANALYZER_H

#include "FeatureGraph.h"
#include <iostream>
#include <algorithm>


using namespace std;

class GraphAnalyzer {


public:	
		
	FeatureGraph G;
	//TODO: Store a heap of triangeles using cpp prioirty_queue

    //constructor
	// 
	// G: FeatureGraph to build on
    GraphAnalyzer(FeatureGraph& G): G(G) {
        vector<Graph_Node> graph = G.getGraph();

        for(int i = 0; i < graph.size(); i++){
            vector<Neighbor_Node*> og_neighbors = graph[i].neighbors;
            for(int j = 0; j < og_neighbors.size(); j++){

                int first_neighbor_id = og_neighbors[j]->node->id;
                int weight1 = og_neighbors[j]->weight;
                Graph_Node neighbor_node = getGraphNode(first_neighbor_id);
                vector<Neighbor_Node*> first_neighbor_neighbors = neighbor_node.neighbors;

                for(int k = 0; k < first_neighbor_neighbors.size(); k++){
                    int neighbor_check = first_neighbor_neighbors[k]->node->id;
                    int weight2 = first_neighbor_neighbors[k]->weight;

                    bool closed = false;
                    bool neither = false;
                    for(int l = 0; l < og_neighbors.size(); l++){
                        if(neighbor_check == graph[i].node->id){
                            neither = true;
                            continue;
                        } else if(neighbor_check == first_neighbor_id){
                            neither = true;
                            continue;
                        } else if(og_neighbors[l]->node->id == neighbor_check){
                            closed = true;

                            vector<int> node_ids;
                            node_ids.push_back(graph[i].node->id);
                            node_ids.push_back(first_neighbor_id);
                            node_ids.push_back(neighbor_check);
                            sort(node_ids.begin(), node_ids.end());

                            int weight = weight1 + weight2 + og_neighbors[l]->weight;

                            Triangle new_closed = Triangle(node_ids, weight);

                            bool duplicate = false;
                            vector<Triangle> temp = closed_triangles;

                            make_heap(temp.begin(), temp.end());

                            for(int m = 0; m < temp.size(); m++){
                                if(temp[m] == new_closed){
                                    duplicate = true;
                                }
                            }
                            if(!duplicate){
                                closed_triangles.push_back(new_closed);
                            }
                        }
                    }
                    if(!closed && !neither){
                        vector<int> node_ids;
                        node_ids.push_back(graph[i].node->id);
                        node_ids.push_back(first_neighbor_id);
                        node_ids.push_back(neighbor_check);
                        sort(node_ids.begin(), node_ids.end());

                        int weight = weight1 + weight2;

                        Triangle new_open = Triangle(node_ids, weight);

                        bool duplicate = false;
                        vector<Triangle> temp = open_triangles;

                        make_heap(temp.begin(), temp.end());

                        for(int m = 0; m < temp.size(); m++){
                            if(temp[m] == new_open){
                                duplicate = true;
                            }
                        }
                        if(!duplicate){
                            open_triangles.push_back(new_open);
                        }
                    }
                }
            }
        }

        make_heap(open_triangles.begin(), open_triangles.end());
        make_heap(closed_triangles.begin(), closed_triangles.end());

	};


    // Insert given node and corresponding features into graph
    // You may assume the number of features is the same as all other nodes in the graph
    void insert(Node n);

    vector<pair<int,int>> Shortest_Path(Graph_Node source_vetrex);

    //Insert given edge into graph
    // You may assume that the edge contains nodes already inserted into the graph
    void insert(Edge e);
    void remove(int node1);
   void remove(int node1, int node2);

   	// Return the diameter of the network.
    int diameter();

    // Return the ratio of open triangles to closed triangles.
    float openClosedTriangleRatio();   

    // Return the top k open triangles ranked by the total weight on their edges.
    // 
    // K: number of triangles to return
    string topKOpenTriangles(int k);

    // Return the top k skilled individuals around a given node based on a given weighting
    //
    // nodeID: Id of seed node
    // k: number of nodes to return
    // w: weight vector
    vector<int> topKNeighbors(int nodeID, int k,  vector<float> w);

    // Return the most skilled individual that does not share an edge with the seed node
    // nodeId: Id of seed node
    // w: weight vector
    int topNonNeighbor(int nodeID, vector<float> w);
    
    // Return the Jacard Index of the top-k neighbors of two seed nodes
    //
    // nodeA: id of first node
    // nodeB id of second node
    // k: number of neighbors to evaluate for each node
    // w: weight vector
    float jacardIndexOfTopKNeighborhoods(int nodeAID, int nodeBID, int k, vector<float> w);

    Graph_Node getGraphNode(int nodeID);

    vector<Triangle> getOpenTriangles();
    vector<Triangle> getClosedTriangles();

private:
    vector<Triangle> open_triangles;
    vector<Triangle> closed_triangles;


};

#endif
