//
// Created by dias4 on 05/05/2023.
//

#include "TSP.h"

#include <utility>
#include <tuple>
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

/*void TSP::setPath(vector<double> paths) {
    this->path = paths;
}*/
