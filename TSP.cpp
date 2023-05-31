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

    ifstream s("../Project2Graphs/Project2Graphs/Extra_Fully_Connected_Graphs/Extra_Fully_Connected_Graphs/"+filename+".csv");
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

void TSP::backtracking(int count, double cost, double& ans, int id,vector<int>& paths,vector<int>& minPath) {
    int n = getGraph()->getNumVertex(); // size of the graph
    auto vertex = graph->findVertex(id);
    if (count == n-1) {
        for (auto edge: vertex->getAdj()) {
            if (edge->getDest()->getId() == 0) {
                cost += edge->getWeight();
                break;
            }
        }
        if(cost < ans){
            ans = cost;
            minPath = paths;
            minPath.push_back(id);
        }
        return;
    }

    for (auto edge : vertex->getAdj()) {
        if(!edge->getDest()->isVisited()){
            edge->getDest()->setVisited(true);
            paths.push_back(vertex->getId());
            backtracking(count + 1, cost + edge->getWeight(), ans, edge->getDest()->getId(),paths,minPath);
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
set<Vertex*> TSP::oddDegree(Graph* mstGraph){
    set<Vertex*> odds;
    for(auto v : mstGraph->getVertexSet()){
        int degree = v->getAdj().size();
        if (degree % 2 == 1) {
            odds.insert(v);
        }
    }
    delete mstGraph;
    return odds;
}
Graph* TSP::oddGraph(set<Vertex*> odds){
    Graph* oddGraph = new Graph();
    double weight;
    for(auto v : odds){
        oddGraph->addVertex(v->getId());
        for(auto u : odds){
            if (u != v) {
                if(u->hasEdge(u,v)){
                    for (auto neighbor : u->getAdj()) {
                        if (neighbor->getDest() == v) {
                            weight = neighbor->getWeight();  // Found an edge
                        }
                    }
                    oddGraph->addEdge(u->getId(), v->getId(), weight);
                }
                else{
                    // Compute the weight between u and v based on the problem domain
                    weight = computeWeight(u, v); // Implement the appropriate weight calculation function
                    // Add an edge between u and v with weight
                    oddGraph->addEdge(u->getId(), v->getId(), weight);
                }
            }
        }
    }

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
void TSP::printMST(Graph* mstGraph) {
    if (mstGraph == nullptr) {
        std::cout << "MST is empty." << std::endl;
        return;
    }

    std::cout << "Minimum Spanning Tree (MST):" << std::endl;

    for (auto v : mstGraph->getVertexSet()) {
        std::cout << "Vertex " << v->getId() << ":" << std::endl;

        for (auto e : v->getAdj()) {
            std::cout << "  Connected to Vertex " << e->getDest()->getId();
            std::cout << " with weight " << e->getWeight() << std::endl;
        }
    }
}
/*
double TSP::DFS(vector<int> &canSol,int id) {
    double min_dist = 0.0;
    stack<int> stack;
    auto vertex = graph->findVertex(id);
    vertex->setVisited(true);
    stack.push(vertex->getId());
    canSol.push_back(0);

    while (!stack.empty()) {
        int curVertex = stack.top();
        vertex = graph->findVertex(curVertex);
        stack.pop();

        // Visit all adjacent vertices of the current vertex
        random_device rd;
        std::mt19937 g(rd());
        std::shuffle(vertex->getAdj().begin(), vertex->getAdj().end(), g);
        for (auto edge : vertex->getAdj()) {
            if (!edge->getDest()->isVisited()) {
                edge->getDest()->setVisited(true);
                min_dist += edge->getWeight();
                canSol.push_back(edge->getDest()->getId());
                stack.push(edge->getDest()->getId());
            }
        }
    }
    return min_dist;
}


vector<int> TSP::simulatedAnnealing(double &ans) {
    double t = 100;
    double tmin = 0.01;
    double alfa = 0.9;
    int id = 0;
    double min_distance = std::numeric_limits<double>::max();
    vector<int> candSol; //= generate_initial_solution(min_distance);
    double energy = DFS(candSol,id);
            //min_distance;
    double eThreshold = 0;
    while(t > tmin && energy > eThreshold){
        //vector<int> newSol = generate_initial_solution(min_distance);
        //double vE = min_distance - energy;
        vector<int> newSol;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> distribution(0, graph->getNumVertex());
        double vE = DFS(newSol,distribution(gen)) - energy;
        if (acceptanceFunction(t,vE)){
            candSol = newSol;
            energy = min_distance;
        }
        t = t/alfa;
    }
    return candSol;
}

bool TSP::acceptanceFunction(double &t, double &vE) {
    random_device rd;
    mt19937  gen(rd());
    uniform_real_distribution<double> dis(0.0,1.0);
    if(vE < 0){
        return true;
    }
    else{
        double r = dis(gen);
        if( r < exp(-vE/t)){
            return true;
        }
        else{
            return false;
        }
    }
}
vector<int> TSP::generate_initial_solution(double& min_distance) {
    int num_cities = graph->getNumVertex();
    std::vector<int> solution(num_cities);

    std::vector<bool> visited(num_cities, false);
    visited[0] = true;
    solution[0] = 0;

    for (int i = 1; i < num_cities; ++i) {
        int current_city = solution[i - 1];
        int nearest_city = -1;

        for (const auto& neighbor : graph->findVertex(current_city)->getAdj()) {
            int city = neighbor->getDest()->getId();
            double distance = neighbor->getWeight();

            if (!visited[city] && distance < min_distance) {
                min_distance = distance;
                nearest_city = city;
            }
        }

        solution[i] = nearest_city;
        visited[nearest_city] = true;
    }

    return solution;
}


vector<int> TSP::nearest_neighbor() {
    int num_cities = graph->getNumVertex();
    std::vector<int> solution;
    std::vector<bool> visited(num_cities, false);

    // Start with the specified city
    int current_city = 0;
    solution.push_back(current_city);
    visited[current_city] = true;

    // Repeat until all cities are visited
    while (solution.size() < num_cities) {
        int nearest_city = -1;
        int min_distance = std::numeric_limits<int>::max();

        // Find the nearest unvisited city
        for (int city = 0; city < num_cities; ++city) {
            if (!visited[city] && graph->findVertex(current_city)->getAdj()[city]->getWeight() < min_distance){//distances[current_city][city] < min_distance) {
                nearest_city = city;
                min_distance = graph->findVertex(current_city)->getAdj()[city]->getWeight();
            }
        }

        // Add the nearest city to the solution
        solution.push_back(nearest_city);
        visited[nearest_city] = true;
        current_city = nearest_city;
    }

    return solution;
}*/