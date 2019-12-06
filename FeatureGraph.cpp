#include <string>
#include <vector>
#include "FeatureGraph.h"
#include "GraphHelper.h"

#include <iostream>


using namespace std;

FeatureGraph::FeatureGraph(int N, int d, vector<Node> nodes, vector<Edge> edges) {
    //TODO
    for(int i = 0; i < nodes.size(); i++){
        Node* node = new Node(nodes[i].id, nodes[i].features);
        Graph_Node graph_node(node);
        graph_node.index = i;
        graph.push_back(graph_node);
    }

    for(int i = 0; i < edges.size(); i++){
        int index_ida = -1;
        int index_idb = -1;
        int weight = edges[i].weight;
        for(int j = 0; j < graph.size(); j++){
            if(graph[j].node->id == edges[i].IdA) {
                index_ida = j;
            }
            if(graph[j].node->id == edges[i].IdB){
                index_idb = j;
            }
        }
        Neighbor_Node* node1 = new Neighbor_Node(graph[index_idb].node, weight);
        Neighbor_Node* node2 = new Neighbor_Node(graph[index_ida].node, weight);

        graph[index_ida].neighbors.push_back(node1);
        graph[index_idb].neighbors.push_back(node2);
    }

//    for(int i = 0; i < table.size(); i++){
//        cout << table[i].node->id << endl;
//        for(int j = 0; j< table[i].neighbors.size(); j++){
//            cout << "neighbor id: " << table[i].neighbors[j]->id << endl;
//        }
//        cout << "next" << endl;
//    }

};

void FeatureGraph::insert(Node node){
    //TODO
    Node* new_node = new Node(node.id, node.features);
    Graph_Node graph_node(new_node);
    graph.push_back(graph_node);
};
    
void FeatureGraph::insert(Edge edge){
    //TODO

    int index_ida = -1;
    int index_idb = -1;
    int weight = edge.weight;

    for(int j = 0; j < graph.size(); j++){
        if(graph[j].node->id == edge.IdA) {
            index_ida = j;
        }
        if(graph[j].node->id == edge.IdB){
            index_idb = j;
        }
    }

    Neighbor_Node* node1 = new Neighbor_Node(graph[index_idb].node, weight);
    Neighbor_Node* node2 = new Neighbor_Node(graph[index_ida].node, weight);

    graph[index_ida].neighbors.push_back(node1);
    graph[index_idb].neighbors.push_back(node2);

};

    void FeatureGraph::erase(int node_id){
        for(int i = 0; i < graph.size(); i++){
            if(graph[i].node->id == node_id){
                graph.erase(graph.begin() + i);
            }
        }
    };
    void FeatureGraph::erase(int id1, int id2){
        int index_ida = -1;
        int index_idb = -1;

        for(int j = 0; j < graph.size(); j++){
            if(graph[j].node->id == id1) {
                index_ida = j;
            }
            if(graph[j].node->id == id2){
                index_idb = j;
            }
        }


        for(int i = 0; i<graph[index_ida].neighbors.size(); i++){
            graph[index_ida].neighbors.erase( graph[index_ida].neighbors.begin() + i);
            i--;
        }

        for(int i = 0; i<graph[index_idb].neighbors.size(); i++){
            graph[index_idb].neighbors.erase( graph[index_idb].neighbors.begin() + i);
            i--;
        }

    };

vector<Graph_Node> FeatureGraph::getGraph(){
    return graph;
}

