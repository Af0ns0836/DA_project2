//
// Created by dias4 on 05/05/2023.
//

#include "TSP.h"
#include "sstream"
#include "fstream"
#include "stack"

TSP::TSP() {
    graph = new Graph();
}

void TSP::readSmallDataSet(const string& filename) {

    ifstream s("../Project2Graphs/Project2Graphs/Toy-Graphs/shipping.csv");
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
        graph->addEdge(origem,destino,distancia);
    }
}

void TSP::readMediumDataSet(const string& filename) {

    ifstream s(filename);
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
        graph->addEdge(origem,destino,distancia);
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
        graph->addEdge(origem,destino,distancia);
    }
}
void TSP::readBigDataSet(const string& filename){
    readBigDataSetNodes(filename + "/nodes.csv");
    readBigDataSetEdges(filename + "/edges.csv");
}

Graph *TSP::getGraph(){
    return graph;
}

void TSP::backtracking(int count, double cost, double ans) {
    //int graph[][V]-> grafo, vector<bool>& v -> vetor com os nos visitados,
    // int currPos -> posicao atual, int n -> numero de vertices, int count, int cost, int& ans -> minium weight of the Hamiltonian Cycle
    // If last node is reached and it has a link
    // to the starting node i.e the source then
    // keep the minimum value out of the total cost
    // of traversal and "ans"
    // Finally return to check for more possible values
    int n = getGraph()->getNumVertex(); // size of the graph
    if (count == n) {
        ans = min(ans, cost + graph->findVertex(0)->getDist());
        return;
    }
    stack<int> paths;
    // BACKTRACKING STEP
    // Loop to traverse the adjacency list
    // of currPos node and increasing the count
    // by 1 and cost by graph[currPos][i] value
    for (auto g : graph->getVertexSet()) {
        // Mark as visited
        g->setVisited(true);
        paths.push(g->getDist());
        backtracking(count + 1, cost + graph->findVertex(g->getId())->getDist(), ans);
        // Mark ith node as unvisited
        g->setVisited(false);
        paths.pop();

    }
}
/*
static Stack<integer> paths = new Stack<>();

modify the for cycle as follows, forget about the comments

for (int i = 0; i < n; i++) {
// nqs nyja nuk eshte e visituar per kete cikel hamiltonian
// dhe nqs ekziston nje lidhje midis nyjes ky ndodhemi (currPos) dhe nyjeve te tjera ne graf graph[currPos][i]
if (v[i] == false && graph[currPos][i] > 0) {

// vizito nyjen
v[i] = true;
// shto ne path nyjen e vizituar
paths.add(i);

//therrit rekurisivisht funksionin

// e vetmja menyre qe te kemi fleksibilitet per
// koston eshte ta ruajme ate si parameter, ne funksion, perkatesisht -> res

// pra vizitojme secilen nyje sipas nje cikli hamiltonian
//dhe gjejme koston e rruges
res = tsp(graph, v, i, n, count + 1,
          cost + graph[currPos][i], res);

// me fundin e nje cikli, ktheji nyjet te pavizituara
// dhe nxjerrim nga stacku qytetet e vizituara
// duke i pergatitur te dy vektoret per ciklin tjeter Hamiltonian
v[i] = false;
paths.pop();
}
}*/
