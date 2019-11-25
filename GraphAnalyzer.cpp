#include "GraphHelper.h"
#include "FeatureGraph.h"
#include "GraphAnalyzer.h"
#include <algorithm>



#include <iostream>


#include <vector>

using namespace std;

bool descending_order(const pair<float,int> &a,
                      const pair<float,int> &b)
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
    vector<Graph_Node> graph = G.getGraph();

    int current_diameter = -1;
    for(int i = 0; i < graph.size(); i++){
        if(graph[i].neighbors.size() == 0){
            continue;
        }
        vector<pair<int, int>> shortest_path = Shortest_Path(graph[i]);
        int current_max = shortest_path[0].first;
        for(int j = 1; j < shortest_path.size(); j++) {
            if (current_max < shortest_path[j].first)
                current_max = shortest_path[j].first;

            if (current_max > current_diameter)
                current_diameter = current_max;
        }

    }
    return current_diameter;
};


float GraphAnalyzer::openClosedTriangleRatio() {
    //TODO
    if(closed_triangles.size() == 0 ){
        return -1;
    }

    float num_open = open_triangles.size();
    float num_closed = closed_triangles.size();

    float ratio =  num_open/num_closed;

    return ratio;
};

string GraphAnalyzer::topKOpenTriangles(int k) {
    //TODO

    //sort in largest to smallest with weights
    string result;
    sort(open_triangles.begin(), open_triangles.end());
    for(int i = open_triangles.size(); i >= 0 ; i--){
        for(int j = 0; j < open_triangles[i].getNodeIds().size(); j++){
            if(j == 2){
                result += to_string(open_triangles[i].getNodeIds()[j]) + ";";
            } else{
                result += to_string(open_triangles[i].getNodeIds()[j]) + ",";
            }

        }
    }

    result = result.substr(0, result.size() - 1);

    return result;
};


vector<int> GraphAnalyzer::topKNeighbors(int nodeID, int k,  vector<float> w) {
    //TODO

    Graph_Node graph_node = getGraphNode(nodeID);
    vector<int> node_ids(0, 0);
    vector<pair<float,int>> node_priority_pair;

    for(int i = 0; i < graph_node.neighbors.size(); i++){
            //calc priority
            float priority = 0;
            vector<float> node_features = graph_node.neighbors[i]->node->features;
            for(int j = 0; j < w.size(); j++){ //may seg fault, make features.size
                priority += w[j] * node_features[j];
            }

            pair<float, int> to_insert = make_pair(priority, graph_node.neighbors[i]->node->id);
            node_priority_pair.push_back(to_insert);

    }

    sort(node_priority_pair.begin(), node_priority_pair.end(), descending_order);

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
    Graph_Node node = getGraphNode(nodeID);

    vector<Neighbor_Node*> neighbor_ids = node.neighbors;
    vector<Graph_Node> graph = G.getGraph();

    for(int j = 0; j < neighbor_ids.size(); j++){
        for(int i = 0; i < graph.size(); i++){
            if(graph[i].node->id == neighbor_ids[j]->node->id){
                graph.erase(graph.begin() + i);
                i--;
                continue;
            }
            if(graph[i].node->id == nodeID){
                graph.erase(graph.begin() + i);
                i--;
            }
        }
    }

    for(int i = 0; i < graph.size(); i++){

    }

    if(graph.size() == 0){
        return -1;
    }

    int max_priority = 0;
    int node_id = 0;

    for(int i = 0; i < graph.size(); i++){
        float priority = 0;
        vector<float> node_features = graph[i].node->features;
        for(int j = 0; j < w.size(); j++){
            priority += w[j] * node_features[j];
        }
        if(priority > max_priority){
            node_id = graph[i].node->id;
            max_priority = priority;
        }
    }


    return node_id;
};


float GraphAnalyzer::jacardIndexOfTopKNeighborhoods(int nodeAID, int nodeBiID, int k, vector<float> w) {
    //TODO
    return 0;
};

Graph_Node GraphAnalyzer::getGraphNode(int nodeID){
    vector<Graph_Node> graph = G.getGraph();
    for(int i = 0; i < graph.size(); i++){
        if(graph[i].node->id == nodeID){
            return graph[i];
        }
    }
    return nullptr;
}

vector<pair<int,int>> GraphAnalyzer::Shortest_Path(Graph_Node source_vetrex) {
    vector<Graph_Node> graph = G.getGraph();

    vector<pair<int, int>> distance;
    vector<pair<int, int>> queue;

    for(int i = 0; i < graph.size(); i++){
        if(graph[i].node->id == source_vetrex.node->id) {
            distance.push_back(make_pair(0, graph[i].node->id));
        }else{
            distance.push_back(make_pair(INT_MAX, graph[i].node->id));
        }
    }

    vector<int> Visited_Before;
    queue.resize(distance.size());
    copy(distance.begin(), distance.end(), queue.begin());

    make_heap(queue.begin(),queue.end());
    sort_heap(queue.begin(),queue.end());

    while(!queue.empty()){
        bool found = false;
        for(int i = 0; i < Visited_Before.size(); i++){
            for(int j = 0; j < queue.size(); j++){
                if( Visited_Before[i] == queue[j].second){
                    queue.erase(queue.begin() + j);
                    sort_heap(queue.begin(),queue.end());
                    found = true;
                }
            }

        }


        pair<int, int> current_pair = queue.front();

        if(found){
            continue;
        } else{
            Visited_Before.push_back(current_pair.second);
        }

        Graph_Node graph_node = getGraphNode(current_pair.second);

        for(int i = 0; i < graph_node.neighbors.size(); i++){
            //make sure not viasited

            int current_id = graph_node.node->id;
            int neighbor_id = graph_node.neighbors[i]->node->id;

            int current_distance;
            int neighbor_distance;

            int neighbor_index;

            for(int j = 0; j < distance.size(); j++){
                if(current_id == distance[j].second){
                    current_distance = distance[j].first;
                }
                if(neighbor_id == distance[j].second){
                    neighbor_index = j;
                    neighbor_distance = distance[j].first;
                }
            }

            if(neighbor_distance > current_distance + graph_node.neighbors[i]->weight){
                distance[neighbor_index].first = current_distance +  graph_node.neighbors[i]->weight;
            }
        }

    }

    return distance;

}

int GraphAnalyzer::numberOpenTriangles(vector <Graph_Node> graph) {

    return 0;
}





