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
void Graph::tabuSearch(std::vector<int> &path, int &totalWeight, int startVertex, long &time)
{
    RandomGenerator random;

    // get greedy path as starting solution
    getGreedyPath(path, totalWeight, 0);

    std::vector<std::vector<int>> tabuList;
    tabuList.reserve(numVertices);

    tabuList.push_back(path);

    // select neighborhood solver based on neighborhood choice
    NeighborhoodSolver *neighborhoodSolver;
    if (neighborhoodChoice == 2)
        neighborhoodSolver = new NeigborhoodInsertSolver();
    else if (neighborhoodChoice == 3)
        neighborhoodSolver = new NeigborhoodInvertSolver();
    else
        neighborhoodSolver = new NeigborhoodSwapSolver();

    int iterationsWithoutImprovement = 0;
    int iterations = 0;
    // number of iterations without improvement after which we diversify
    const int diversificationInterval = 5;
    const int tabuTenure = 20000;

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
            if (neighborCost < pathCost)
            {
                if (std::find(tabuList.begin(), tabuList.end(), neighbor) == tabuList.end())
                {
                    path = neighbor;
                    pathCost = neighborCost;
                    iterationsWithoutImprovement = 0; // Reset counter
                }
            }
        }

        // if current solution is better than best solution so far, update best solution
        if (pathCost < totalWeight)
        {
            totalWeight = pathCost;
            time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - currentTime).count();
        }

        // add current solution to tabu list
        tabuList.push_back(path);
        // if tabu list is full, remove oldest solution
        if (tabuList.size() > tabuTenure)
            tabuList.erase(tabuList.begin());

        // increment iterations without improvement
        iterationsWithoutImprovement++;
        iterations++;
    }

    // delete neighborhood solver
    delete neighborhoodSolver;
}

void Graph::simulatedAnnealing(std::vector<int> &path, int &totalWeight, int startVertex, long &time)
{
    RandomGenerator random;
    double currTemperature = temperature;
    std::cout << "Initial temperature: " << currTemperature << std::endl;
    // get greedy path as starting solution
    getGreedyPath(path, totalWeight, random.generateRandomInt(0, numVertices - 1));

    std::cout << "total weight: " << totalWeight << std::endl;
    // select neighborhood solver based on neighborhood choice
    NeighborhoodSolver *neighborhoodSolver = new NeigborhoodInsertSolver();

    int iterations = 0;

    auto currentTime = std::chrono::high_resolution_clock::now();
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

        std::cout << "path cost: " << pathCost << std::endl;
        std::cout << "neighbor cost: " << neighborCost << std::endl;

        // if neighbor is better than current solution, update current solution
        double probability = exp(-1*(neighborCost - pathCost) / currTemperature);

        if (neighborCost < pathCost || random.generateRandomInt(0, 100) < probability * 100)
        {
            path = neighbor;
            pathCost = neighborCost;
            // std::cout << pathCost << std::endl;
            if (pathCost < totalWeight)
            {
                totalWeight = pathCost;
                time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - currentTime).count();
                std::cout << " iterations: " << iterations << "temperature: " << currTemperature << " time: " << time << " total weight: " << totalWeight << std::endl;
            }
        }

        // if current solution is better than best solution so far, update best solution

        // update temperature

        currTemperature *= alpha;
        iterations++;
    }

    // delete neighborhood solver
    delete neighborhoodSolver;
}

double Graph::calculateInitialTemperature()
{
    RandomGenerator random;
    double totalDifference = 0;
    int numIterations = 1000;

    for (int i = 0; i < numIterations; i++)
    {
        std::vector<int> path;
        int totalWeight = 0;

        getGreedyPath(path, totalWeight, 0);

        std::vector<std::vector<int>> neighborhood = NeigborhoodInsertSolver().getNeighborhood(path);
        std::vector<int> neighbor = neighborhood[random.generateRandomInt(0, neighborhood.size() - 1)];

        int pathCost = getPathCost(path);
        int neighborCost = getPathCost(neighbor);

        totalDifference += abs(neighborCost - pathCost);
    }

    double averageDifference = totalDifference / numIterations;

    // Adjust this factor based on your problem characteristics
    double initialTemperatureFactor = 0.5;

    // Adjust this factor based on your desired acceptance probability at the beginning
    double acceptanceProbabilityFactor = 0.8;

    double initialTemperature = averageDifference / (-log(acceptanceProbabilityFactor) * initialTemperatureFactor);
    return initialTemperature;
}
