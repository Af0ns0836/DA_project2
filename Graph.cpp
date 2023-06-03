// By: Gonçalo Leão

#include <unordered_set>
#include "Graph.h"

int Graph::getNumVertex() const {
    return vertexSet.size();
}

std::vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
Vertex * Graph::findVertex(const int &id) const {
    for (auto v : vertexSet)
        if (v->getId() == id)
            return v;
    return nullptr;
}

/*
 * Finds the index of the vertex with a given content.
 */
int Graph::findVertexIdx(const int &id) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getId() == id)
            return i;
    return -1;
}
/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
bool Graph::addVertex(const int &id) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex(id));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
bool Graph::addEdge(const int &sourc, const int &dest, double w) const {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}
/**
 * Resets the nodes' visited, path and cost values.
 */
void Graph::resetNodes() const {
    for (auto v: getVertexSet()) {
        v->setVisited(false);
        v->setPath(nullptr);

        for (Edge *e: v->getAdj()) {
            e->setVisited(false);
        }
    }
}
void Graph::setNumberEdges(int n) {
    this->numberEdges = n;
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

int Graph::getNumberEdges() {
    return this->numberEdges;
}
void Graph::bfs() const {
    // Get the source vertex
    auto s = findVertex(0);
    if (s == nullptr) {
        return;
    }

    // Set that no vertex has been visited yet
    for (auto v : vertexSet) {
        v->setVisited(false);
    }

    // Perform the actual BFS using a queue
    std::queue<Vertex *> q;
    q.push(s);
    s->setVisited(true);
    while (!q.empty()) {
        auto v = q.front();
        q.pop();
        for (auto & e : v->getAdj()) {
            auto w = e->getDest();
            if ( ! w->isVisited()) {
                q.push(w);
                w->setVisited(true);
            }
        }
    }
}
bool Graph::isConnectedGraph() const {
    if (getNumVertex() == 0) {
        // An empty graph is considered connected
        return true;
    }

    // Perform breadth-first search to check graph connectivity
    std::unordered_set<int> visited; // Track visited vertices
    std::queue<int> q;

    // Choose an arbitrary starting vertex
    auto vertex = findVertex(0);
    q.push(0);
    visited.insert(0);

    while (!q.empty()) {
        int currentVertex = q.front();
        q.pop();

        // Visit all adjacent vertices
        for (const auto& edge : vertex->getAdj()) {
            int nextVertex = edge->getDest()->getId();

            if (visited.count(nextVertex) == 0) {
                // Mark the vertex as visited and add it to the queue
                visited.insert(nextVertex);
                q.push(nextVertex);
            }
        }
    }

    // Check if all vertices have been visited
    return visited.size() == getNumVertex();
}

Edge* Graph::getNonBridgeEdge(int vertex) const {
    auto v = findVertex(vertex);
    for (const auto& edge : v->getAdj()) {
        // Temporarily remove the edge from the multigraph
        v->deleteEdge(edge);

        //Check if the removal of the edge disconnects the graph
        bool isConnected = isConnectedGraph();

        // Restore the edge to the multigraph
        addEdge(edge->getOrig()->getId(),edge->getDest()->getId(),edge->getWeight());

        // If the graph remains connected, return the non-bridge edge
        if (isConnected) {
            return edge;
        }
    }

    return nullptr;  // No non-bridge edge found
}
int Graph::countConnectedComponents() const {
    std::vector<bool> visited(getNumVertex(), false);
    int count = 0;
    for(auto vertex : getVertexSet()){
        if(!vertex->isVisited()){
            bfs();
            count++;
        }
    }

    return count;
}
bool Graph::isBridge(Edge* edge) {
    // Store the original connectivity information
    int initialComponents = countConnectedComponents();

    // Remove the edge (u, v)
    //findVertex(edge->getOrig()->getId())->removeEdge(edge->getDest()->getId());
    findVertex(edge->getOrig()->getId())->deleteEdge(edge);

    // Check the connectivity after removing the edge
    int finalComponents = countConnectedComponents();

    // Restore the edge (u, v)
    addEdge(edge->getOrig()->getId(), edge->getDest()->getId(),edge->getWeight());

    // If the number of components increases, the edge is a bridge
    return finalComponents > initialComponents;
}

void Graph::MST(std::vector<int>& parent) {

    std::vector<double> key(getNumVertex(), std::numeric_limits<double>::max()); //Tracks values of the
    std::vector<bool> inMST(getNumVertex(), false); //Tracks vertices already in the MST

    // Root value
    int startVertex = 0;

    //Start key vector with the key value
    key[startVertex] = 0.0;

    for (int count = 0; count < getNumVertex() - 1; ++count) {
        int u = minKey(key, inMST);
        inMST[u] = true;

        // Update key and parent index of adjacent vertices
        for (auto edge : findVertex(u)->getAdj()) {
            int v = edge->getDest()->getId();
            double weight = edge->getWeight();


            if (!inMST[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
            }
        }
    }

    // Print the MST
    std::cout << "Minimum Spanning Tree:" << std::endl;
    for (int i = 1; i < getNumVertex(); ++i) {
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

int Graph::minKey(std::vector<double> &key, std::vector<bool> &inMST){
    double min = std::numeric_limits<double>::max();
    int minValue = -1; //Minimum value possible
    int numVertices = key.size();

    for (int v = 0; v < numVertices; ++v) {
        if (!inMST[v] && key[v] < min) {
            min = key[v];
            minValue = v;
        }
    }

    return minValue;
}


double Graph::totalDistance(const std::vector<int> &path) {
    //TODO
}

Graph::~Graph() {
    for (auto vertex : vertexSet) {
        delete vertex;
    }
}




