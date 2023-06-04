//
// Created by dias4 on 05/05/2023.
//

#include "TSP.h"
#include "algorithm"
#include <random>
#include "sstream"
#include "fstream"
#include "stack"


TSP::TSP() {
    graph = new Graph();
}

void TSP::readSmallDataSet(const string& filename) {

    ifstream s("../Project2Graphs/Project2Graphs/Toy-Graphs/"+filename+".csv");
    string line;
    int origem,destino;
    double distancia;
    getline(s,line);
    while(getline(s,line)){
        stringstream ss(line);
        getline(ss,line,',');
        origem = stoi(line);
        getline(ss,line,',');
        destino = stoi(line);
        getline(ss,line,',');
        distancia = stod(line);
        graph->addVertex(origem);
        auto v1 = graph->findVertex(destino);
        if(v1 == nullptr){
            graph->addVertex(destino);
        }
        //graph->addEdge(origem,destino,distancia);
        graph->addBidirectionalEdge(origem,destino,distancia);
    }
}

void TSP::readMediumDataSet(const string& filename) {

    ifstream s("../Project2Graphs/Project2Graphs/Extra_Fully_Connected_Graphs/Extra_Fully_Connected_Graphs/edges_"+filename+".csv");
    string line;
    int origem,destino;
    double distancia;
    while(getline(s,line)){
        stringstream ss(line);
        getline(ss,line,',');
        origem = stoi(line);
        getline(ss,line,',');
        destino = stoi(line);
        getline(ss,line,',');
        distancia = stod(line);
        graph->addVertex(origem);
        auto v1 = graph->findVertex(destino);
        if(v1 == nullptr){
            graph->addVertex(destino);
        }
        graph->addBidirectionalEdge(origem,destino,distancia);
    }
}


void TSP::readBigDataSetNodes(const string& filename) {
    ifstream s("../Project2Graphs/Project2Graphs/Real-World-Graphs/graph1/nodes.csv");
    string line;
    int id;
    double latitude,longitude;
    getline(s,line);
    while(getline(s,line)){
        stringstream ss(line);
        getline(ss,line,',');
        id = stoi(line);
        getline(ss,line,',');
        latitude = stod(line);
        getline(ss,line,',');
        longitude = stod(line);
        graph->addVertex(id);
        graph->findVertex(id)->setLatLon(latitude,longitude);
    }
}

void TSP::readBigDataSetEdges(const string &filename) {
    ifstream s("../Project2Graphs/Project2Graphs/Real-World-Graphs/graph1/edges.csv");
    string line;
    int origem,destino;
    double distancia;
    getline(s,line);
    while(getline(s,line)){
        stringstream ss(line);
        getline(ss,line,',');
        origem = stoi(line);
        getline(ss,line,',');
        destino = stoi(line);
        getline(ss,line,',');
        distancia = stod(line);
        graph->addBidirectionalEdge(origem,destino,distancia);
    }
}
void TSP::readBigDataSet(const string& filename){
    readBigDataSetNodes(filename + "/nodes.csv");
    readBigDataSetEdges(filename + "/edges.csv");
}

Graph *TSP::getGraph(){
    return graph;
}

void TSP::backtracking(int count, double cost, double& ans, int id, vector<int>& paths, vector<int>& minPath) {
    int n = getGraph()->getNumVertex(); // size of the graph
    auto vertex = graph->findVertex(id);
    if (count == n) {
        for (auto edge : vertex->getAdj()) {
            if (edge->getDest()->getId() == 0) {
                cost += edge->getWeight();
                break;
            }
        }
        if (cost < ans) {
            ans = cost;
            minPath = paths;
            minPath.push_back(id);
        }
        return;
    }
    for (auto edge : vertex->getAdj()) {
        if (!edge->getDest()->isVisited()) {
            edge->getDest()->setVisited(true);
            paths.push_back(vertex->getId());
            backtracking(count + 1, cost + edge->getWeight(), ans, edge->getDest()->getId(), paths, minPath);
            paths.pop_back();
            edge->getDest()->setVisited(false);
        }
    }

}


double TSP::getPaths()
{
    double ans = numeric_limits<double>::max();
    graph->findVertex(0)->setVisited(true);
    vector<int> path;
    vector<int> minPath;
    backtracking(0,0.0,ans,0,path,minPath);
    for(auto p : minPath){
        cout << p << "-->";
    }
    cout << endl;
    return ans;
}
Graph* TSP::MST() {
    if (graph->getVertexSet().empty()) {
        return nullptr;
    }

    Graph* mstGraph = new Graph();

    // Reset auxiliary info
    for (auto v : graph->getVertexSet()) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }

    // Start with an arbitrary vertex
    Vertex* s = graph->getVertexSet().front();
    s->setDist(0);

    // Initialize priority queue
    MutablePriorityQueue<Vertex> q;
    q.insert(s);

    // Process vertices in the priority queue
    while (!q.empty()) {
        auto v = q.extractMin();
        v->setVisited(true);
        mstGraph->addVertex(v->getId());
        if (v->getPath() != nullptr) {
            // Add the edge to the MST graph
            mstGraph->addEdge(v->getPath()->getOrig()->getId(),v->getPath()->getDest()->getId(),v->getPath()->getWeight());
        }

        for (auto& e : v->getAdj()) {
            Vertex* w = e->getDest();
            if (!w->isVisited()) {
                auto oldDist = w->getDist();
                if (e->getWeight() < oldDist) {
                    w->setDist(e->getWeight());
                    w->setPath(e);
                    if (oldDist == INF) {
                        q.insert(w);
                    }
                    else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }

    return mstGraph;
}
double TSP::computeWeight(Vertex* u, Vertex* v) {

    double lat1 = u->getLat();
    double lon1 = u->getLon();
    double lat2 = v->getLat();
    double lon2 = v->getLon();

    // Use the Haversine formula to calculate the distance
    double distance = haversine(lat1, lon1, lat2, lon2); // Implement the Haversine formula

    return distance;
}
double TSP::haversine(double lat1,double lon1,double lat2,double lon2){
    double earth_R = 6371.0;
    double lat1R = lat1 * M_PI / 180.0;
    double lon1R = lon1 * M_PI / 180.0;
    double lat2R = lat2 * M_PI / 180.0;
    double lon2R = lon2 * M_PI / 180.0;
    double dlat = lat2R - lat1R;
    double dlon = lon2R - lon1R;
    double a = sin(dlat/2) * sin(dlat/2) + cos(lat1R) * cos(lat2R) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = earth_R * c;
    return d;
}

void TSP::nearestNeighborTSP() {
    int n = graph->getNumVertex();
    vector<Vertex*> tour;
    tour.reserve(n);

    // Start with an arbitrary vertex (e.g., the first vertex)
    Vertex* current = graph->findVertex(0);
    current->setVisited(true);
    tour.push_back(current);
    double totalWeight = 0.0;

    while (tour.size() < n) {
        Vertex* nearestNeighbor = nullptr;
        double minDistance = numeric_limits<double>::max();

        // Find the nearest unvisited neighbor
        for (const auto& edge : current->getAdj()) {
            auto neighbor = edge->getDest();
            if (!neighbor->isVisited()) {
                if (edge->getWeight() < minDistance) {
                    minDistance = edge->getWeight();
                    nearestNeighbor = neighbor;
                }
            }
        }

        if (nearestNeighbor) {
            // Mark the nearest neighbor as visited and add it to the tour
            nearestNeighbor->setVisited(true);
            tour.push_back(nearestNeighbor);

            // Add the weight of the edge to the total weight
            totalWeight += minDistance;

            // Set the nearest neighbor as the current vertex for the next iteration
            current = nearestNeighbor;
        }
    }

    // Add the weight of the last edge (back to the starting vertex)
    totalWeight += current->getAdj()[graph->findVertex(0)->getId()]->getWeight();

    cout << "Total Weight: " << totalWeight << 'm' << endl;
    cout << "Tour: ";
    for (auto vertex : tour) {
        cout << vertex->getId() << " ";
    }
    cout << endl;
}

double TSP::triangularApproximation() {
    // PRIM'S ALGORITHM - Find the Minimum Spamming Tree
    vector<int> par(graph->getNumVertex(), -1);
    graph->MST(par);


    // DFS - Find the order of visited vertices
    vector<bool> visitedVertices(graph->getNumVertex(), false);
    vector<int> pathValues;
    stack<int> cityStack;
    graph->DFS(0, par, visitedVertices, cityStack, pathValues);

    // PREORDER - Print the order of visited cities
    cout << "Path of visited vertices: ";
    for (int x = 0; x < pathValues.size(); ++x) {
        cout << pathValues[x];
        if (x != pathValues.size() - 1)
            cout << " -> ";
    }
    cout << " -> 0"<< endl;

    //TODO - Find the distance
    double total_distance = graph->totalDistance(pathValues);

    //return total_distance;
    return 0;
}
