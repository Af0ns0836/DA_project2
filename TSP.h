//
// Created by dias4 on 05/05/2023.
//

#ifndef DA_PROJECT2_TSP_H
#define DA_PROJECT2_TSP_H

#include "Graph.h"
#include "set"

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
    Graph* oddGraph(set<Vertex*> odds);
    double computeWeight(Vertex* u, Vertex* v);
    double haversine(double lat1,double lon1,double lat2,double lon2);
    void christofides();
    /*vector<int> simulatedAnnealing(double& ans);
    bool acceptanceFunction(double &t, double &vE);
    vector<int> generate_initial_solution(double& min_distance);
    vector<int> nearest_neighbor();
    double DFS(vector<int> &canSol,int id);*/
    double triangularApproximation();

};

#endif //DA_PROJECT2_TSP_H
