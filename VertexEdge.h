// By: Gonçalo Leão

#ifndef DA_TP_CLASSES_VERTEX_EDGE
#define DA_TP_CLASSES_VERTEX_EDGE

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include "MutablePriorityQueue.h"

class Edge;

#define INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

class Vertex {
public:
    Vertex(int id);
    bool operator<(Vertex & vertex) const; // // required by MutablePriorityQueue

    int getId() const;
    std::vector<Edge *> getAdj() const;
    bool isVisited() const;
    double getDist() const;
    Edge *getPath() const;
    std::vector<Edge *> getIncoming() const;

    void setId(int info);
    void setVisited(bool visited);
    void setDist(double dist);
    void setPath(Edge *path);
    void setLatLon(double lat, double lon);
    double getLat();
    double getLon();
    Edge * addEdge(Vertex *dest, double w);
    bool removeEdge(int destID);
    void deleteEdge(Edge *edge);
    bool hasEdge(Vertex *vertex1, Vertex *vertex2);
    friend class MutablePriorityQueue<Vertex>;
protected:
    int id;                // identifier
    std::vector<Edge *> adj;  // outgoing edges
    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    double dist = 0;
    Edge *path = nullptr;
    double lat = 0;
    double lon = 0;
    std::vector<Edge *> incoming; // incoming edges
    int queueIndex = 0; 		// required by MutablePriorityQueue and UFDS

};

/********************** Edge  ****************************/

class Edge {
public:
    Edge(Vertex *orig, Vertex *dest, double w);
    Vertex * getDest() const;
    double getWeight() const;
    bool isSelected() const;
    Vertex * getOrig() const;
    Edge *getReverse() const;
    double getFlow() const;
    void setSelected(bool selected);
    void setReverse(Edge *reverse);
    void setVisited(bool visited);
protected:
    Vertex * dest; // destination vertex
    double weight; // edge weight, can also be used for capacity
    // auxiliary fields
    bool selected = false;
    // used for bidirectional edges
    Vertex *orig;
    Edge *reverse = nullptr;
    bool visited = false;

};

#endif /* DA_TP_CLASSES_VERTEX_EDGE */