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

double TSP::backtracking(int count, double cost, double& ans, int id,vector<double> paths) {
    int n = getGraph()->getNumVertex(); // size of the graph
    auto vertex = graph->findVertex(id);
    if (count == n) {
        if (!vertex->isVisited()) {
            vertex->setVisited(true);
            for (auto edge: vertex->getAdj()) {
                if(!edge->getOrig()->isVisited()){
                    cost+= edge->getWeight();
                }
                paths.push_back(edge->getOrig()->getId());
                setPath(paths);
            }
        }
        ans = min(ans, cost);
        return ans;
    }

    if (!vertex->isVisited()) {
        vertex->setVisited(true);
        for (auto edge : vertex->getAdj()) {
            paths.push_back(edge->getOrig()->getId());
            setPath(paths);
            backtracking(count + 1, cost + edge->getWeight(), ans, id + 1,paths);
            paths.pop_back(); // Remove the last added weight from paths
        }
        vertex->setVisited(false);
    }

    return ans;
}


vector<double> TSP::getPaths()
{
    double maxd = numeric_limits<double>::max();
    cout << backtracking(1,0.0,maxd,0,paths) << endl;
    return paths;
}

vector<double> TSP::setPath(vector<double> paths) {
    return this->paths = paths;
}
