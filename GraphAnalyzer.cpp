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
        for(int j = 1; j < shortest_path.size(); j++){
            if(current_max < shortest_path[j].first)
                current_max = shortest_path[j].first;

        if(current_max > current_diameter)
            current_diameter = current_max;
    }



    }
    return current_diameter;
};


float GraphAnalyzer::openClosedTriangleRatio() {
    //TODO

    vector<Graph_Node> graph = G.getGraph();

    int num_open = 0;
    int num_closed = 0;
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
                        num_closed++;

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
                        sort_heap(temp.begin(), temp.end());

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

                    num_open++;

                    bool duplicate = false;
                    vector<Triangle> temp = open_triangles;

                    make_heap(temp.begin(), temp.end());
                    sort_heap(temp.begin(), temp.end());

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

    num_open /= 2;
    num_closed /= 6;

    float ratio = (double) num_open/num_closed;



//    cout << "num open " << open_triangles << endl;
//    cout << "num closed " << num_closed << endl;
//
//    for(int i = 0; i < open_triangles.size(); i++){
//        for(int j = 0; j < open_triangles[i].getNodeIds().size(); j++){
//            cout << open_triangles[i].getNodeIds()[j];
//        }
//        cout << " weight: " << open_triangles[i].getWeight() << endl;
//    }

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





