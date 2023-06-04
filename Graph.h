// By: Gonçalo Leão

#ifndef DA_TP_CLASSES_GRAPH
#define DA_TP_CLASSES_GRAPH

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <stack>
#include "MutablePriorityQueue.h"

#include "VertexEdge.h"

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
    std::vector<Vertex *> getVertexSet() const;
    void MST(std::vector<int>& parent);
    void DFS(int current, const std::vector<int> &parent, std::vector<bool> &visited, std::stack<int> &cityStack, std::vector<int> &path);
    int minKey(std::vector<double> &key, std::vector<bool> &inMST);
    double totalDistance(const std::vector<int> &path);
    void resetNodes() const;
protected:
    std::vector<Vertex *> vertexSet;    // vertex set
    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const int &id) const;
};


#endif /* DA_TP_CLASSES_GRAPH */