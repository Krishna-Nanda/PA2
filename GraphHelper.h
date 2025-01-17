#ifndef HELPER_H
#define HELPER_H

#include <vector>
using namespace std;

struct Node {
    // NOTE: Do not edit node struct
    int id;
    vector<float> features;
    
    Node(int id, vector<float> features): id(id), features(features) {}
    Node(const Node &n2) { id = n2.id; features=n2.features;}
};


struct Edge {
    // NOTE: Do not edit Edge struct
    int IdA, IdB, weight;
    
    Edge(int IdA, int IdB, int weight): IdA(IdA), IdB(IdB), weight(weight) {}
};

class Triangle {
    // TODO make a Triangle class with appropriate fields and methods

public:

    // TODO make appropriate constuctor
    Triangle(vector<int> nodes, int w) {
        nodes_ids = nodes;
        weight = w;
    }
    
    // Operator overloading for storage in priority queue
    // returns true iff t2 is greater than t1. 
    //
    // Note: Must be transitive
    //      This means if t1<t2 and t2<t3 than t1< t3
    bool operator== (Triangle  const &other) const {
        //TODO

        for(int i = 0; i < 3; i++){
            if(nodes_ids[i] != other.nodes_ids[i]){
                return false;
            }
        }
        return true;
    }

        bool operator < (Triangle const &other) const {
//        //TODO
        int other_weight = other.weight;
        return weight < other_weight;
    }

    //insert triangle function

    vector<int> getNodeIds(){
        return nodes_ids;
    }
    int getWeight(){
        return weight;
    }
private:
    vector<int> nodes_ids;
    int weight;

};



#endif

    




    
