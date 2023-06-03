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
    int origem,destino,nEdges = 0;
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
        nEdges++;
    }
    graph->setNumberEdges(nEdges);
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
int TSP::findAlternativeNonBridgeEdge(int currentVertex, const Graph& multigraph) {
    for (const auto& edge : multigraph.findVertex(currentVertex)->getAdj()) {
        // Temporarily remove the edge from the multigraph
        multigraph.findVertex(currentVertex)->deleteEdge(edge);

        // Check if the removal of the edge creates a bridge
        if (!multigraph.isConnectedGraph()) {
            // Restore the removed edge and continue to the next edge
            multigraph.addEdge(edge->getOrig()->getId(),edge->getDest()->getId(),edge->getWeight());
            continue;
        }

        // The removed edge is not a bridge, so return the other vertex of the edge
        int otherVertex = edge->getDest()->getId();
        return otherVertex;
    }

    // No alternative non-bridge edge found
    return -1; // Or any appropriate indication of failure
}
bool TSP::isValidMatching(const vector<pair<int, int>>& matching) {
    // Check if the matching is valid
    unordered_map<int, int> vertices;
    for (const auto& edge : matching) {
        int u = edge.first;
        int v = edge.second;
        if (vertices.count(u) || vertices.count(v)) {
            return false;
        }
        vertices[u] = 1;
        vertices[v] = 1;
    }
    return true;
}
bool TSP::isPerfectMatching(const vector<pair<int, int>>& matching) {
    // Check if the matching is a perfect matching
    unordered_map<int, int> vertices;
    for (const auto& edge : matching) {
        int u = edge.first;
        int v = edge.second;
        if (vertices.count(u) || vertices.count(v)) {
            return false;
        }
        vertices[u] = 1;
        vertices[v] = 1;
    }
    return vertices.size() == graph->getNumVertex();
}
vector<pair<int, int>> TSP::bruteForcePerfectMatching(const Graph* graph) {
    vector<pair<int, int>> matching;
    vector<pair<int, int>> allEdges;

    // Generate all possible combinations of edges
    for (const auto& entry : graph->getVertexSet()) {
        int u = entry->getId();
        for (const auto& v : entry->getAdj()) {
            allEdges.emplace_back(u, v->getDest()->getId());
        }
    }

    // Iterate over all possible matchings
    int n = graph->getNumVertex();
    for (int r = n / 2; r <= n; ++r) {  // Consider matchings of size n/2 to n
        vector<bool> chosen(allEdges.size(), false);
        fill(chosen.end() - r, chosen.end(), true);

        do {
            matching.clear();
            for (int i = 0; i < allEdges.size(); ++i) {
                if (chosen[i]) {
                    matching.push_back(allEdges[i]);
                }
            }
            if (isValidMatching(matching) && isPerfectMatching(matching)) {
                return matching;
            }
        } while (next_permutation(chosen.begin(), chosen.end()));
    }

    return {};  // No perfect matching found
}
void TSP::constructEulerianCircuit(Graph* &multigraph) {
    vector<int> circuit; // Store the Eulerian circuit

    // Step 1: Choose a starting vertex
    int startVertex = 0; // Choose any vertex as the starting vertex

    // Step 2: Perform Fleury's algorithm
    performFleuryAlgorithm(startVertex, reinterpret_cast<Graph &>(multigraph), circuit);
    for(int v : circuit){
        cout << v << " ";
    }
}
void TSP::performFleuryAlgorithm(int currentVertex, Graph& multigraph, vector<int>& circuit) {
    while (multigraph.getNumberEdges() > 0) {
        // Step 3: Choose an edge from the current vertex that is not a bridge
        Edge* chosenEdge = multigraph.getNonBridgeEdge(currentVertex);

        // Step 4: Mark the chosen edge as visited
        //multigraph.findVertex(currentVertex)->removeEdge(chosenEdge->getDest()->getId());
        multigraph.findVertex(currentVertex)->deleteEdge(chosenEdge);
        // Step 5: Add the current vertex to the circuit
        circuit.push_back(currentVertex);

        // Move to the other vertex of the chosen edge
        int nextVertex = chosenEdge->getDest()->getId();

        // If the chosen edge was a bridge, find an alternative non-bridge edge
        if (multigraph.isBridge(chosenEdge)) {
            multigraph.addEdge(chosenEdge->getOrig()->getId(),chosenEdge->getDest()->getId(),chosenEdge->getWeight());
            // Restore the removed bridge edge

            // Find an alternative non-bridge edge using a depth-first search
            nextVertex = findAlternativeNonBridgeEdge(currentVertex, multigraph);
        }

        // Move to the next vertex
        currentVertex = nextVertex;
    }

    // Step 6: Add the final vertex to complete the circuit
    circuit.push_back(currentVertex);
}

void TSP::christofides() {
    //Creation of the odd degree Graph from the MST
    auto oddG = oddGraph(oddDegree(MST()));
    //Step to build the Eulerian multigraph
    auto perfect_matching = bruteForcePerfectMatching(oddG);
    auto multigraph = MST(); // Start with the MST

    for (const auto& pair : perfect_matching) {
        int source = pair.first;
        int vertexDestination = pair.second;

        // Retrieve the actual vertices and weight associated with the IDs
        double weight = haversine(multigraph->findVertex(source)->getLat(),multigraph->findVertex(source)->getLon(),
        multigraph->findVertex(vertexDestination)->getLat(),multigraph->findVertex(vertexDestination)->getLon());
        // Add the edge to the multigraph
        multigraph->addEdge(source, vertexDestination, weight);
    }

    // At this point, the multigraph contains duplicated edges and has even degree vertices
    // Proceed to step 5 to construct an Eulerian circuit in the multigraph
    constructEulerianCircuit(multigraph);
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
