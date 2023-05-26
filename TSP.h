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
    vector<double> paths;

public:

    TSP();
    void readSmallDataSet(const string& filename);
    void readMediumDataSet(const string& filename);
    void readBigDataSet(const string& filename);
    void readBigDataSetNodes(const string& filename);
    void readBigDataSetEdges(const string& filename);
    double backtracking(int count, double cost, double& ans, int id, vector<double> paths);
    vector<double> setPath(vector<double> paths);
    vector<double> getPaths();
    Graph * getGraph();
};

#endif //DA_PROJECT2_TSP_H
