//
// Created by dias4 on 05/05/2023.
//

#include "TSP.h"
#include "sstream"
#include "fstream"

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

void TSP::readBigDataSet(const string& filename) {
    ifstream s("../Project2Graphs/Project2Graphs/Real-World-Graphs/graph1/nodes.csv");
    string line;
    int origem,destino;
    double distancia;

    while(s.peek() != EOF){
        getline(s,line,s.widen(','));
        origem = stoi(line);
        graph->addVertex(origem);
        //graph->addEdge(origem,destino,distancia);
    }
}

Graph *TSP::getGraph(){
    return graph;
}