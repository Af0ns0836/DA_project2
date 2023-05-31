//
// Created by dias4 on 05/05/2023.
//

#ifndef DA_PROJECT2_TSP_H
#define DA_PROJECT2_TSP_H

#include "Graph.h"

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
    //void setPath(vector<double> paths);
    double getPaths();
    double triangularApproximation();

};

#endif //DA_PROJECT2_TSP_H
