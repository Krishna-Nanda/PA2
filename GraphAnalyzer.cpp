#include "GraphHelper.h"
#include "FeatureGraph.h"
#include "GraphAnalyzer.h"
#include <algorithm>

#include <iostream>


#include <vector>

using namespace std;

bool descending_order(const pair<int,int> &a,
                      const pair<int,int> &b)
{
    return (a.first > b.first);
}

void GraphAnalyzer::insert(Node n) {
    G.insert(n);
    // TODO Adjust calculations for ratio of open triangles and topKtriangles
};

void GraphAnalyzer::insert(Edge e) {
    G.insert(e);
    // TODO Adjust calculations for ratio of open triangles and topKtriangles
};

int GraphAnalyzer::diameter() {
    //TODO
    return 2;
};


float GraphAnalyzer::openClosedTriangleRatio() {
    //TODO
    return .5;
};

string GraphAnalyzer::topKOpenTriangles(int k) {
    //TODO
    return "2,3,4";
};


vector<int> GraphAnalyzer::topKNeighbors(int nodeID, int k,  vector<float> w) {
    //TODO

    List_Node list_node = getListNode(nodeID);
    vector<int> node_ids(0, 0);
    vector<pair<float,int>> node_priority_pair;

    for(int i = 0; i < list_node.neighbors.size(); i++){
            //calc priority
            float priority = 0;
            vector<float> node_features = list_node.neighbors[i]->features;
            for(int j = 0; j < w.size(); j++){ //may seg fault, make features.size
                priority += w[j] * node_features[j];
            }

            pair<float, int> to_insert = make_pair(priority, list_node.neighbors[i]->id);
            node_priority_pair.push_back(to_insert);

    }

    sort(node_priority_pair.begin(), node_priority_pair.end(), descending_order);

//    for(int i = 0; i < node_priority_pair.size(); i++){
//        cout << "Priority is " << node_priority_pair[i].first << " at " <<  node_priority_pair[i].second << endl;
//    }

    for(int i = 0; i < k; i++){
        if(node_priority_pair.size() > i){
            node_ids.push_back(node_priority_pair[i].second);
        } else {
            node_ids.push_back(0);
        }
    }
    return node_ids;
};


int GraphAnalyzer::topNonNeighbor(int nodeID, vector<float> w) {
    //TODO
    return 1;
};


float GraphAnalyzer::jacardIndexOfTopKNeighborhoods(int nodeAID, int nodeBiID, int k, vector<float> w) {
    //TODO
    return 0;
};

List_Node GraphAnalyzer::getListNode(int nodeID){
    vector<List_Node> graph = G.getTable();
    for(int i = 0; i < graph.size(); i++){
        if(graph[i].node->id == nodeID){
            return graph[i];
        }
    }
    return nullptr;
}



