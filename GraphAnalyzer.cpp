#include "GraphHelper.h"
#include "FeatureGraph.h"
#include "GraphAnalyzer.h"

using namespace std;


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
    vector<int> node_ids(k, 0);

    map<int,float> node_priority_pair;
    for(int i = 0; i < NeighborMap[nodeID].size(); i++){
        if(NeighborMap[nodeID][i] == 1){
            //calc priority
            float priority = 0;
            for(int j = 0; j < w.size(); j++){
                vector<float> features = getNode(nodeID)->features;
                priority += w[j] * features[i];
            }
            node_priority_pair[nodeID] = priority;
        }
    }

    vector<pair<int, float>> vector;
    map<int, float> :: iterator it;
    for (it=node_priority_pair.begin(); it!=node_priority_pair.end(); it++){
        vector.push_back(make_pair(it->first, it->second));
    }

    sort(vector.begin(), vector.end(), greater<int>());

    for(int i = 0; i < k; i++){
        node_ids.push_back(vector[i]->first);
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



