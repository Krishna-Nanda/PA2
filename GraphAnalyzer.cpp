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

void GraphAnalyzer::remove(int node1) {
    Graph_Node graphNode = getGraphNode(node1);
    G.erase(node1);
};

void GraphAnalyzer::remove(int node1, int node2) {
    Graph_Node graphNode = getGraphNode(node1);
    Graph_Node graphNode2 = getGraphNode(node2);

    if(graphNode.neighbors.size() == 1 && graphNode2.neighbors.size() == 1){
        G.erase(node1, node2);
        return;
    }

    if(graphNode.neighbors.size() == 1){

        for(int i = 0; i < open_triangles.size(); i++){
            bool found = false;
            for(int j = 0; j < 2; j++){
                if(node1 == open_triangles[i].getNodeIds()[j]){
                    found = true;
                }
            }
            if(found){
                open_triangles.erase(open_triangles.begin() + i);
                i--;
            }
        }

        G.erase(node1, node2);
        return;
    } else if(graphNode2.neighbors.size() == 1){

        for(int i = 0; i < open_triangles.size(); i++){
            bool found = false;
            for(int j = 0; j < 2; j++){
                if(node2 == open_triangles[i].getNodeIds()[j]){
                    found = true;
                }
            }
            if(found){
                open_triangles.erase(open_triangles.begin() + i);
                i--;
            }
        }

        G.erase(node1, node2);
        return;
    }

    //more than one neighbor each
    vector<int> common_neighbors;
    vector<pair<int,int>> weights;

    for(int i = 0; i < graphNode.neighbors.size(); i++){
        bool in_common = false;
        if(graphNode.neighbors[i]->node->id == graphNode2.node->id){
            continue;
        }
        for(int j = 0; j < graphNode2.neighbors.size(); j++) {
            if (graphNode.neighbors[i]->node->id == graphNode2.neighbors[j]->node->id) {
                in_common = true;
                common_neighbors.push_back(graphNode.neighbors[i]->node->id);
                weights.push_back(make_pair(graphNode.neighbors[i]->weight, graphNode2.neighbors[j]->weight));
            }
        }

    }

//    for(int i = 0; i < graphNode2.neighbors.size(); i++){
//        bool in_common = false;
//        if(graphNode2.neighbors[i]->node->id == graphNode.node->id){
//            continue;
//        }
//        for(int j = 0; j < graphNode.neighbors.size(); j++){
//            if(graphNode.neighbors[j]->node->id == graphNode2.neighbors[i]->node->id){
//                common_neighborsB.push_back(make_pair(graphNode.neighbors[j]->node->id,graphNode2.neighbors[i]->weight));
//            }
//        }
//        if(!in_common){
//            if(graphNode2.neighbors[i]->node->id == graphNode.node->id){
//                continue;
//            }
//           // uncommon_neighborsB.push_back(graphNode2.neighbors[i]->node->id);
//        }
//
//    }

    for(int j = 0; j < common_neighbors.size(); j++){
        vector<int> nodes;
        nodes.push_back(node1);
        nodes.push_back(node2);
        nodes.push_back(common_neighbors[j]);

        sort(nodes.begin(), nodes.end());

        int weight = weights[j].second + weights[j].first;

        Triangle new_open = Triangle(nodes,weight);

        open_triangles.push_back(new_open);
        push_heap(open_triangles.begin(), open_triangles.end());

        for(int i = 0; i < closed_triangles.size(); i++){
            if(new_open == closed_triangles[i]){
                closed_triangles.erase(closed_triangles.begin() + i);
                i--;
            }
        }

    }

    G.erase(node1, node2);

    make_heap(open_triangles.begin(), open_triangles.end());
    make_heap(closed_triangles.begin(), closed_triangles.end());
};

void GraphAnalyzer::insert(Edge e) {
    G.insert(e);
    // TODO Adjust calculations for ratio of open triangles and topKtriangles

    Graph_Node ida_node = getGraphNode(e.IdA);
    Graph_Node idb_node = getGraphNode(e.IdB);

    if(ida_node.neighbors.size() == 1 && idb_node.neighbors.size() == 1){
        return;
    }
    if(ida_node.neighbors.size() == 1){

        for(int i = 0; i < idb_node.neighbors.size(); i++){

            if(idb_node.neighbors[i]->node->id == e.IdA){
                continue;
            }

            vector<int> nodes;
            nodes.push_back(e.IdA);
            nodes.push_back(e.IdB);
            nodes.push_back(idb_node.neighbors[i]->node->id);

            int weight = e.weight + idb_node.neighbors[i]->weight;

            Triangle new_open = Triangle(nodes, weight);

            open_triangles.push_back(new_open);
            push_heap(open_triangles.begin(), open_triangles.end());

        }
        return;

    } else if(idb_node.neighbors.size() == 1){

        for(int i = 0; i < ida_node.neighbors.size(); i++){

            if(ida_node.neighbors[i]->node->id == e.IdB){
                continue;
            }

            vector<int> nodes;
            nodes.push_back(e.IdA);
            nodes.push_back(ida_node.neighbors[i]->node->id);
            nodes.push_back(e.IdB);

            int weight = e.weight + ida_node.neighbors[i]->weight;

            Triangle new_open = Triangle(nodes, weight);

            open_triangles.push_back(new_open);
            push_heap(open_triangles.begin(), open_triangles.end());

        }
        return;
    }

    vector<int> common_neighbors;
    vector<pair<int, int>> uncommon_neighborsA;
    vector<pair<int, int>> uncommon_neighborsB;

    vector<pair<int, int>> weights;

    for(int i = 0; i < ida_node.neighbors.size(); i++){
        bool in_common = false;
        for(int j = 0; j < idb_node.neighbors.size(); j++){

            if(ida_node.neighbors[i]->node->id == idb_node.neighbors[j]->node->id){

                in_common = true;
                common_neighbors.push_back(ida_node.neighbors[i]->node->id);
                weights.push_back(make_pair(ida_node.neighbors[i]->weight, idb_node.neighbors[j]->weight));

            }
        }
        if(!in_common){
            if(ida_node.neighbors[i]->node->id == idb_node.node->id){
                continue;
            }
            uncommon_neighborsA.push_back(make_pair(ida_node.neighbors[i]->node->id, ida_node.neighbors[i]->weight));
        }

    }

    for(int i = 0; i < idb_node.neighbors.size(); i++){
        bool in_common = false;
        for(int j = 0; j < ida_node.neighbors.size(); j++){
            if(ida_node.neighbors[j]->node->id == idb_node.neighbors[i]->node->id){
                in_common = true;
            }
        }
        if(!in_common){
            if(idb_node.neighbors[i]->node->id == ida_node.node->id){
                continue;
            }
            uncommon_neighborsB.push_back(make_pair(idb_node.neighbors[i]->node->id, idb_node.neighbors[i]->weight));
        }

    }

    for(unsigned long i = 0; i < common_neighbors.size(); i++){

        //closed
        vector<int> nodes;
        nodes.push_back(e.IdA);
        nodes.push_back(e.IdB);
        nodes.push_back(common_neighbors[i]);

        int weight = e.weight + weights[i].first + weights[i].second;

        Triangle new_closed = Triangle(nodes, weight);

        closed_triangles.push_back(new_closed);
        push_heap(closed_triangles.begin(), closed_triangles.end());

        //open
        for(unsigned long j = 0; j < open_triangles.size(); j++){
            if(new_closed == open_triangles[j]){
                open_triangles.erase(open_triangles.begin() + j);
                j--;
                make_heap(open_triangles.begin(), open_triangles.end());
            }
        }

    }

        for(unsigned long i = 0; i < uncommon_neighborsA.size(); i++){

            vector<int> nodes;
            nodes.push_back(e.IdB);
            nodes.push_back(e.IdA);
            nodes.push_back(uncommon_neighborsA[i].first);

            int weight = e.weight + uncommon_neighborsA[i].second;

            Triangle new_open = Triangle(nodes, weight);

            open_triangles.push_back(new_open);
            push_heap(open_triangles.begin(), open_triangles.end());
        }

    for(unsigned long i = 0; i < uncommon_neighborsB.size(); i++){

        vector<int> nodes;
        nodes.push_back(e.IdA);
        nodes.push_back(e.IdB);
        nodes.push_back(uncommon_neighborsB[i].first);

        int weight = e.weight + uncommon_neighborsB[i].second;

        Triangle new_open = Triangle(nodes, weight);

        open_triangles.push_back(new_open);
        push_heap(open_triangles.begin(), open_triangles.end());
    }



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
    float num_open = open_triangles.size();
    float num_closed = closed_triangles.size();

    if(num_closed <= 0){
        return -1;
    }

    float ratio =  num_open/num_closed;

    return ratio;
};

string GraphAnalyzer::topKOpenTriangles(int k) {
    //TODO

    //sort in largest to smallest with weights
    vector<Triangle> temp_open_triangles;
    for(int i = 0; i < open_triangles.size(); i++){
        temp_open_triangles.push_back(open_triangles[i]);
    }

    make_heap(temp_open_triangles.begin(), temp_open_triangles.end());

    string result = "";

    if(k > temp_open_triangles.size() || temp_open_triangles.empty()){
        return result;
    }

    for(int i = 0; i < k; i++){
        for(int j = 0; j < temp_open_triangles.front().getNodeIds().size(); j++){
            if(j == 2){
                result += to_string(temp_open_triangles.front().getNodeIds()[j]) + ";";
            } else{
                result += to_string(temp_open_triangles.front().getNodeIds()[j]) + ",";
            }

        }

        pop_heap(temp_open_triangles.begin(), temp_open_triangles.end());
        temp_open_triangles.pop_back();
//        cout << "------";
//        for(int i = 0; i < temp_open_triangles.size(); i++){
//            for(int j = 0; j < temp_open_triangles[i].getNodeIds().size(); j++){
//                cout << temp_open_triangles[i].getNodeIds()[j] << " ";
//            }
//            cout << endl;
//        }

    }

    result = result.substr(0, result.size() - 1);

    return result;
};


vector<int> GraphAnalyzer::topKNeighbors(int nodeID, int k,  vector<float> w) {
    //TODO
    Graph_Node graph_node = getGraphNode(nodeID);
    vector<int> node_ids(0, 0);
    vector<pair<float,int>> node_priority_pair(0);

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

    if(graph.size() - 1 == neighbor_ids.size()){
        return -1;
    }

    for(int j = 0; j < neighbor_ids.size(); j++){
        for(int i = 0; i < graph.size(); i++){
            if(graph[i].node->id == neighbor_ids[j]->node->id){
                graph.erase(graph.begin() + i);
                i--;
            } else if(graph[i].node->id == nodeID){
                graph.erase(graph.begin() + i);
                i--;
            }
        }
    }

    double max_priority = 0.0;
    int node_id = -1;

    for(unsigned long i = 0; i < graph.size(); i++){
        if(graph[i].node->id == nodeID){
            continue;
        }
        double priority = 0.0;
        vector<float> node_features = graph[i].node->features;
        for(int j = 0; j < node_features.size(); j++){
            priority += w[j] * node_features[j];
        }
        if(priority > max_priority){
            node_id = graph[i].node->id;
            max_priority = priority;
        }
    }

    if(max_priority == 0){
        node_id = graph[graph.size()-1].node->id;
    }

    return node_id;
};


float GraphAnalyzer::jacardIndexOfTopKNeighborhoods(int nodeAID, int nodeBiID, int k, vector<float> w) {
    //TODO
    vector<int> top_neighborA = topKNeighbors(nodeAID,k,w);
    vector<int> top_neighborB = topKNeighbors(nodeBiID,k,w);

    vector<int> common(0);
    vector<int> unique(0);

    for(int i = 0; i < top_neighborA.size(); i++){
        for(int j = 0; j < top_neighborB.size(); j++){
            if(top_neighborA[i] == top_neighborB[j]){
                    common.push_back(top_neighborA[i]);
                }
            }
        }

    double size1 = common.size();
    double size2 = top_neighborA.size() + top_neighborB.size() - common.size();

    if( size2 == 0){
        return 1;
    }

    float result = size1 / size2;

    return result;
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
            distance.push_back(make_pair(INT32_MAX, graph[i].node->id));
        }
    }

    vector<int> Visited_Before;
    queue.resize(distance.size());
    copy(distance.begin(), distance.end(), queue.begin());

    make_heap(queue.begin(),queue.end(), descending_order);

    while(!queue.empty()){

        pair<int, int> current_pair = queue.front();

        Visited_Before.push_back(current_pair.second);

        pop_heap(queue.begin(), queue.end());
        queue.pop_back();

        Graph_Node graph_node = getGraphNode(current_pair.second);

        for(int i = 0; i < graph_node.neighbors.size(); i++){
            //make sure not viasited

            //update distance
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

            //update queue
            int neighbor_index_queue;

            for(int j = 0; j < queue.size(); j++){
                if(neighbor_id == queue[j].second){
                    neighbor_index_queue = j;
                }
            }
            if(neighbor_distance > current_distance + graph_node.neighbors[i]->weight){
                distance[neighbor_index].first = current_distance +  graph_node.neighbors[i]->weight;
                queue[neighbor_index_queue].first = current_distance +  graph_node.neighbors[i]->weight;
            }
        }

        make_heap(queue.begin(),queue.end(), descending_order);

    }

    return distance;

}

vector<Triangle> GraphAnalyzer::getOpenTriangles(){
    return open_triangles;
}

vector<Triangle> GraphAnalyzer::getClosedTriangles(){
    return closed_triangles;
}





