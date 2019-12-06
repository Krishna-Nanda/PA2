#include "GraphHelper.h"
#include "FeatureGraph.h"
#include "GraphAnalyzer.h"
#include <map>
#include <iostream>
using namespace std;

int main() {
    
    vector<Node> nodes  {
       Node(1, vector<float> { 10, 10}),
       Node(2,vector<float> { 20, 20}),
       Node(3, vector<float> { 30, 30}),
       Node(4, vector<float> { 40, 40})
//       Node(5, vector<float> { 50, 50})
//       Node(5, vector<float> {50,50}),
//       Node(6, vector<float> (60,60))
            };
    
    vector<Edge> edges {Edge(1,2, 10), Edge(2, 3, 9), Edge(3, 4, 5), Edge(4, 1, 32), Edge(2, 4, 3), Edge(3, 1, 62)};//, Edge(4,1, 3), Edge(2, 5, 5), Edge(6, 1, 3), Edge(6, 3, 3)};
  
    int d = 2;

    FeatureGraph graph = FeatureGraph(4, d, nodes, edges);
    GraphAnalyzer analyzer = GraphAnalyzer(graph);


    cout << "BEFORE" << endl;
    cout << analyzer.getOpenTriangles().size() << endl;
    cout << analyzer.getClosedTriangles().size()  << endl;

    cout << analyzer.openClosedTriangleRatio() << "\n";
    analyzer.remove(4, 3);
    analyzer.remove(4);

    cout << "After" << endl;
    cout << analyzer.getOpenTriangles().size() << endl;
    cout << analyzer.getClosedTriangles().size()  << endl;

    cout << analyzer.openClosedTriangleRatio() << "\n";
//for(int i = 0; i < analyzer.getOpenTriangles().size(); i++){
//    cout << "triangle ";
//    for(int j = 0; j < analyzer.getOpenTriangles()[i].getNodeIds().size(); j++){
//        cout << analyzer.getOpenTriangles()[i].getNodeIds()[j] << " ";
//    }
//    cout << analyzer.getOpenTriangles()[i].getWeight();
//    cout << endl;
//}

//    cout << analyzer.diameter() << "\n";

//    cout << analyzer.openClosedTriangleRatio() << "\n";
//
//    cout << analyzer.topKOpenTriangles(3) << "\n";

    
//    int newNodeID = 7;
//    vector<float> newFeatures {3, 3};
//    Node newNode = Node(newNodeID, newFeatures);
//
//    int newNodeID2 = 8;
//    Node newNode2 = Node(newNodeID2, newFeatures);

//
//    cout << "BEFORE" << endl;
//    cout << analyzer.getOpenTriangles().size() << endl;
//    cout << analyzer.getClosedTriangles().size()  << endl;

//    int newNodeID = 7;
//    vector<float> newFeatures {3, 3};
//    Node newNode = Node(newNodeID, newFeatures);

//    analyzer.insert(newNode);
//    analyzer.insert(Edge(7, 4, 32));

//    cout << analyzer.topKOpenTriangles(4) << endl;
//    cout << analyzer.openClosedTriangleRatio() << "\n";
//
//    analyzer.insert(newNode);
//    analyzer.insert(newNode2);



//


//    cout << "BETWEEN" << endl;
//    cout << analyzer.getOpenTriangles().size() << endl;
//    cout << analyzer.getClosedTriangles().size()  << endl;
//
//    analyzer.insert(Edge(4, 8, 100));
//
    cout << "AFTER" << endl;
    cout << analyzer.getOpenTriangles().size() << endl;
    cout << analyzer.getClosedTriangles().size()  << endl;
    cout << analyzer.topKOpenTriangles(3) << endl;
//
//
//    analyzer.insert(Edge(4, 3, 1));
//    analyzer.insert(Edge(7, 8, 1));
//
//    cout << "AFTER2" << endl;
//    cout << analyzer.getOpenTriangles().size() << endl;
//    cout << analyzer.getClosedTriangles().size()  << endl;
//
//    cout << "AFTER" << endl;
//    cout << analyzer.getOpenTriangles().size() << endl;
//    cout << analyzer.getClosedTriangles().size()  << endl;
//
//
//
//    cout << analyzer.topKOpenTriangles(3) << endl;
//    cout << analyzer.openClosedTriangleRatio() << endl;

    vector<float> weights{.5, .5};
    vector<int> neighbors = analyzer.topKNeighbors(2, 3, weights);
    
    for(auto i = neighbors.begin(); i != neighbors.end(); ++i)
        cout << *i << ",";
    cout << "\n";

    cout << analyzer.topNonNeighbor(3, weights) << "\n";

    cout << analyzer.jacardIndexOfTopKNeighborhoods(1, 2, 2, weights);
    return 0;
}
