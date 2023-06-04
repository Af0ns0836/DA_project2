// By: Gonçalo Leão

#include <cmath>
#include "Graph.h"

int Graph::getNumVertex() const {
    return vertexSet.size();
}

std::unordered_map<int, Vertex*> Graph::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
Vertex * Graph::findVertex(const int &id) const {
    for (auto v : vertexSet)
        if (v.second->getId() == id) {
            return v.second;
        }
    return nullptr;
}

/*
 * Finds the index of the vertex with a given content.
 */
int Graph::findVertexIdx(const int &id) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (findVertex(id)->getId() == id)
            return i;
    return -1;
}
/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
bool Graph::addVertex(const int &id) {
    if (findVertex(id) != nullptr) {
        return false;
    }
    vertexSet[id] = new Vertex(id);
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
bool Graph::addEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}
bool Graph::addBidirectionalEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

void Graph::MST(std::vector<int>& parent) {


    std::vector<double> key(getNumVertex(), std::numeric_limits<double>::max()); //Tracks values of the
    std::vector<bool> inMST(getNumVertex(), false); //Tracks vertices already in the MST

    // Root value
    int startVertex = 0;

    //Start key vector with the key value
    key[startVertex] = 0.0;

    // Priority queue to store vertices with their corresponding key values
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    pq.push(std::make_pair(0.0, startVertex));

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (inMST[u])
            continue;

        inMST[u] = true;

        // Update key and parent index of adjacent vertices
        for (auto edge : findVertex(u)->getAdj()) {
            int v = edge->getDest()->getId();
            double weight = edge->getWeight();

            if (!inMST[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
                pq.push(std::make_pair(weight, v));
            }
        }
    }
    std::vector<std::pair<int, int>> mst;
    // Print the MST
    std::cout << "Minimum Spanning Tree:" << std::endl;
    for (int i = 1; i < getNumVertex(); ++i) {
        mst.push_back(std::make_pair(parent[i], i));
        std::cout << parent[i] << " - " << i << std::endl;
    }

}

void Graph::DFS(int current, const std::vector<int> &parent, std::vector<bool> &visited, std::stack<int> &cityStack, std::vector<int> &path) {
    visited[current] = true;
    cityStack.push(current);

    while (!cityStack.empty()) {
        int city = cityStack.top();
        cityStack.pop();

        // Process the city or print its order
        path.push_back(city);

        for (int neighbor = 0; neighbor < parent.size(); ++neighbor) {
            if (parent[neighbor] == city && !visited[neighbor]) {
                visited[neighbor] = true;
                cityStack.push(neighbor);
            }
        }
    }
}


double Graph::totalDistance(const std::vector<int> &path) {
    double totalDistance = 0.0;

    // Calculate the total distance
    for (int i = 0; i < path.size() - 1; ++i) {
        int v1 = path[i];
        int v2 = path[i + 1];

        if(!check_if_nodes_are_connected(v1, v2)){
            totalDistance = haversine(findVertex(v1)->getLat(), findVertex(v1)->getLon(), findVertex(v2)->getLat(), findVertex(v2)->getLon());
            continue;
        }

        for (auto edge : findVertex(v1)->getAdj()) {
            if (edge->getDest()->getId() == v2) {
                totalDistance += edge->getWeight();
                break;
            }
        }
    }

    //add final distance to total distance
    int final_city = path.back();
    if(!check_if_nodes_are_connected(final_city, path[0])){
        totalDistance += haversine(findVertex(final_city)->getLat(), findVertex(final_city)->getLon(), findVertex(path[0])->getLat(), findVertex(path[0])->getLon());
    }
    else{
        for(auto edge : findVertex(final_city)->getAdj()){
            if(edge->getDest()->getId() == path[0]){
                totalDistance += edge->getWeight();
                break;
            }
        }
    }

    return totalDistance;
}



bool Graph::check_if_nodes_are_connected(int v1, int v2){
    int index_v1;
    //iterate through vertices to find index of v1

    for(auto it = vertexSet.begin(); it != vertexSet.end(); ++it){
        if(it->second->getId() == v1){
            index_v1 = it->first;
            break;
        }
    }

    for(auto edge : findVertex(index_v1)->getAdj()){
        if(edge->getDest()->getId() == v2){
            return true;
        }
    }
    return false;
}


double Graph::degrees_to_radians(double degrees){
    return degrees * M_PI / 180;
}

double Graph::haversine(double lat1, double lon1, double lat2, double lon2){
    if (lat1 == 0 && lon1 == 0 && lat2 == 0 && lon2 == 0) {
        return 0.0;
    }

    double lat1Rad = degrees_to_radians(lat1);
    double lon1Rad = degrees_to_radians(lon1);
    double lat2Rad = degrees_to_radians(lat2);
    double lon2Rad = degrees_to_radians(lon2);

    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;

    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
               std::cos(lat1Rad) * std::cos(lat2Rad) *
               std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = RADIUS * c;

    return distance;
}


