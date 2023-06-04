//
// Created by dias4 on 05/05/2023.
//

#ifndef DA_PROJECT2_TSP_H
#define DA_PROJECT2_TSP_H

#include "Graph.h"
#include "set"
#include "unordered_map"

using namespace std;

class TSP{

private:
    Graph* graph;

public:

    TSP();
    void readSmallDataSet(const string& filename);
    void readMediumDataSet(const string& filename);
    void readBigDataSet(const string& filename);
    void readBigDataSetNodes(const string& filename);
    void readBigDataSetEdges(const string& filename);
    Graph * getGraph();
    void backtracking(int count, double cost, double& ans, int id,vector<int>& paths,vector<int>& minPath);
    double getPaths();
    Graph* MST();
    void printMST(Graph* mstGraph);
    set<Vertex*> oddDegree(Graph* mstGraph);
    //Graph* oddGraph(set<Vertex*> odds);
    double computeWeight(Vertex* u, Vertex* v);
    double haversine(double lat1,double lon1,double lat2,double lon2);
    void christofides();
    int findAlternativeNonBridgeEdge(int currentVertex, const Graph& multigraph);
    double triangularApproximation();
    bool isPerfectMatching(const vector<pair<int, int>>& matching);
    bool isValidMatching(const vector<pair<int, int>>& matching);
    void constructEulerianCircuit(Graph* &multigraph);
    vector<pair<int, int>> bruteForcePerfectMatching(set<Vertex*> odds);
    void performFleuryAlgorithm(int currentVertex, Graph& multigraph, vector<int>& circuit);
};

#endif //DA_PROJECT2_TSP_H
