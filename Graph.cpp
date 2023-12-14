#include "Graph.h"
#include <algorithm>
#include <limits>
#include <random>
// graph constructor
Graph::Graph(int numVertices)
{
    this->numVertices = numVertices;
    this->numEdges = 0;
    adjacencyMatrix.resize(numVertices, std::vector<int>(numVertices, -1));
    greedyPath.reserve(numVertices);
    stopTime = 300;
}

// graph destructor
Graph::~Graph()
{
}

// adding edges
void Graph::addEdge(int u, int v, int weight)
{
    if (u >= 0 && u < numVertices && v >= 0 && v < numVertices)
    {
        adjacencyMatrix[u][v] = weight;
        numEdges++;
    }
}

// getting edge's weight
int Graph::getEdgeWeight(int u, int v) const
{
    if (u >= 0 && u < numVertices && v >= 0 && v < numVertices)
        return adjacencyMatrix[u][v];
    else
        return -1;
}
// returns number of vertices
int Graph::getNumVertices() const
{
    return numVertices;
}

// returns number of edges
int Graph::getNumEdges() const
{
    return numEdges;
}

// returns stop time
int Graph::getStopTime() const
{
    return stopTime;
}

int Graph::getNeighborhoodChoice() const
{
    return neighborhoodChoice;
}

double Graph::getTemperature() const
{
    return temperature;
}

double Graph::getAlpha() const
{
    return alpha;
}

// setting stop time
void Graph::setStopTime(int stopTime)
{
    this->stopTime = stopTime;
}

// setting neighborhood choice
void Graph::setNeighborhoodChoice(int neighborhoodChoice)
{
    this->neighborhoodChoice = neighborhoodChoice;
}

void Graph::setTemperature(double temperature)
{
    this->temperature = temperature;
}

void Graph::setAlpha(double alpha)
{
    this->alpha = alpha;
}

// converting graph into string version
std::string Graph::toString() const
{
    std::string output = "";
    for (int i = 0; i < numVertices; i++)
    {
        for (int j = 0; j < numVertices; j++)
        {
            output += std::to_string(adjacencyMatrix[i][j]) + " ";
        }
        output += "\n";
    }
    return output;
}

// returns adjacency matrix
std::vector<std::vector<int>> Graph::getAdjecencyMatrix() const
{
    return adjacencyMatrix;
}

int Graph::getPathCost(const std::vector<int> &path) const
{
    int cost = 0;
    for (int i = 0; i < path.size() - 1; i++)
        cost += adjacencyMatrix[path[i]][path[i + 1]];
    return cost;
}

// greedy algorithm
void Graph::getGreedyPath(std::vector<int> &path, int &totalWeight, int startVertex)
{
    std::vector<bool> visited(numVertices, false);
    int nextVertex = -1;
    int minWeight = INT_MAX;
    int currentVertex = startVertex;

    path.push_back(currentVertex);
    visited[currentVertex] = true;
    // iterate until all vertices are visited
    while (path.size() < numVertices)
    {
        // iterate through all vertices
        for (int i = 0; i < numVertices; i++)
        {
            // if vertex is not visited and there is an edge between current vertex and i and weight of that edge is smaller than minWeight
            if (i != currentVertex && !visited[i] && adjacencyMatrix[currentVertex][i] != -1 && adjacencyMatrix[currentVertex][i] < minWeight)
            {
                minWeight = adjacencyMatrix[currentVertex][i];
                nextVertex = i;
            }
        }
        // if there is a vertex to visit
        if (nextVertex != -1)
        {
            path.push_back(nextVertex);
            visited[nextVertex] = true;
            totalWeight += minWeight;
            currentVertex = nextVertex;
            nextVertex = -1;
            minWeight = INT_MAX;
        }
    }
    // add weight of edge between last and first vertex
    totalWeight += adjacencyMatrix[currentVertex][startVertex];
    path.push_back(startVertex);
    greedyPath = path;
}

// tabu search algorithm
void Graph::tabuSearch(std::vector<int> &path, int &totalWeight, int startVertex)
{   
    // get greedy path as starting solution
    getGreedyPath(path, totalWeight, startVertex);

    RandomGenerator random;
    std::vector<std::vector<int>> tabuList;
    tabuList.reserve(numVertices);

    // select neighborhood solver based on neighborhood choice
    NeighborhoodSolver *neighborhoodSolver;
    if (neighborhoodChoice == 2)
        neighborhoodSolver = new NeigborhoodInsertSolver();
    else if (neighborhoodChoice == 3)
        neighborhoodSolver = new NeigborhoodInvertSolver();
    else
        neighborhoodSolver = new NeigborhoodSwapSolver();

    int iterationsWithoutImprovement = 0;

    // number of iterations without improvement after which we diversify
    const int diversificationInterval = 20;

    auto currentTime = std::chrono::high_resolution_clock::now();

    // run until stop time is reached
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - currentTime).count() < stopTime)
    {
        // get neighborhood of current solution
        std::vector<std::vector<int>> neighborhood = neighborhoodSolver->getNeighborhood(path);

        // diversify if there is no improvement for diversificationInterval iterations
        if (iterationsWithoutImprovement >= diversificationInterval)
        {
            neighborhood = random.shuffleMatrix(neighborhood); // Shuffle neighborhood
            path = neighborhood[0];
            iterationsWithoutImprovement = 0; // Reset counter
        }

        // calculate cost of current solution
        int pathCost = getPathCost(path);

        // find best solution in neighborhood
        for (auto &neighbor : neighborhood)
        {
            int neighborCost = getPathCost(neighbor);

            // if neighbor is better than current solution and is not in tabu list
            if (neighborCost < pathCost && std::find(tabuList.begin(), tabuList.end(), neighbor) == tabuList.end())
            {
                path = neighbor;
                pathCost = neighborCost;
                iterationsWithoutImprovement = 0; // Reset counter
            }
        }

        // if current solution is better than best solution so far, update best solution
        // and clear tabu list
        if (pathCost < totalWeight)
        {
            totalWeight = pathCost;
            tabuList.clear();
        }

        // add current solution to tabu list
        tabuList.push_back(path);

        // if tabu list is full, remove oldest solution
        if (tabuList.size() > numVertices)
            tabuList.erase(tabuList.begin());

        // increment iterations without improvement
        iterationsWithoutImprovement++;
    }

    // delete neighborhood solver
    delete neighborhoodSolver;
}

void Graph::simulatedAnnealing(std::vector<int> &path, int &totalWeight, int startVertex)
{
    // get greedy path as starting solution
    getGreedyPath(path, totalWeight, startVertex); 

    RandomGenerator random;

    auto currentTime = std::chrono::high_resolution_clock::now();

    NeighborhoodSolver *neighborhoodSolver;
    if (neighborhoodChoice == 2)
        neighborhoodSolver = new NeigborhoodInsertSolver();

    // run until stop time is reached
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - currentTime).count() < stopTime)
    {
        // get neighborhood of current solution
        std::vector<std::vector<int>> neighborhood = neighborhoodSolver->getNeighborhood(path);

        // select random neighbor
        std::vector<int> neighbor = neighborhood[random.generateRandomInt(0, neighborhood.size() - 1)];

        // calculate cost of current solution and neighbor
        int pathCost = getPathCost(path);
        int neighborCost = getPathCost(neighbor);

        // if neighbor is better than current solution, update current solution
        if (neighborCost < pathCost)
        {
            path = neighbor;
            pathCost = neighborCost;
        }
        // if neighbor is worse than current solution, update current solution with probability
        // based on temperature
        else
        {
            double probability = exp((pathCost - neighborCost) / temperature);
            if (random.generateRandomInt(0, 100) < probability * 100)
            {
                path = neighbor;
                pathCost = neighborCost;
            }
        }

        // if current solution is better than best solution so far, update best solution
        if (pathCost < totalWeight)
            totalWeight = pathCost;

        // update temperature
        temperature *= alpha;
    }

    // delete neighborhood solver
    delete neighborhoodSolver;
}
