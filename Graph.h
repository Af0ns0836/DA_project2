// By: Gonçalo Leão

#ifndef DA_TP_CLASSES_GRAPH
#define DA_TP_CLASSES_GRAPH

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <stack>
#include <unordered_map>
#include "MutablePriorityQueue.h"
#include "VertexEdge.h"
#include <cmath>

#define RADIUS (double) 6371000.0

class Graph {
public:
    ~Graph();
    /*
    * Auxiliary function to find a vertex with a given ID.
    */
    Vertex *findVertex(const int &id) const;
    /*
     *  Adds a vertex with a given content or info (in) to a graph (this).
     *  Returns true if successful, and false if a vertex with that content already exists.
     */
    bool addVertex(const int &id);

    /*
     * Adds an edge to a graph (this), given the contents of the source and
     * destination vertices and the edge weight (w).
     * Returns true if successful, and false if the source or destination vertex does not exist.
     */
    bool addEdge(const int &sourc, const int &dest, double w) const;
    bool addBidirectionalEdge(const int &sourc, const int &dest, double w);
    int getNumVertex() const;
    std::unordered_map<int, Vertex*> getVertexSet() const;
    void MST(std::vector<int>& parent);
    void DFS(int current, const std::vector<int> &parent, std::vector<bool> &visited, std::stack<int> &cityStack, std::vector<int> &path);
    double totalDistance(const std::vector<int> &path);
    void resetNodes() const;
    bool check_if_nodes_are_connected(int v1, int v2);
    double toRadians(double degrees);
    double haversine(double lat1, double lon1, double lat2, double lon2);


protected:
    std::unordered_map<int, Vertex*> vertexSet;    // vertex set
    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const int &id) const;
};


#endif /* DA_TP_CLASSES_GRAPH */