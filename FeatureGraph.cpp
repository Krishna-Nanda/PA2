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
        List_Node list_node(node);
        table.push_back(list_node);
    }

    for(int i = 0; i < edges.size(); i++){
        int index_ida = -1;
        int index_idb = -1;
        for(int j = 0; j < table.size(); j++){
            if(table[j].node->id == edges[i].IdA) {
                index_ida = j;
            }
            if(table[j].node->id == edges[i].IdB){
                index_idb = j;
            }
        }

        table[index_ida].neighbors.push_back(table[index_idb].node);
        table[index_idb].neighbors.push_back(table[index_ida].node);
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
    List_Node list_node(new_node);
    table.push_back(list_node);
};
    
void FeatureGraph::insert(Edge edge){
    //TODO

    int index_ida = -1;
    int index_idb = -1;

    for(int j = 0; j < table.size(); j++){
        if(table[j].node->id == edge.IdA) {
            index_ida = j;
        }
        if(table[j].node->id == edge.IdB){
            index_idb = j;
        }
    }

    table[index_ida].neighbors.push_back(table[index_idb].node);
    table[index_idb].neighbors.push_back(table[index_ida].node);

};

vector<List_Node> FeatureGraph::getTable(){
    return table;
}

