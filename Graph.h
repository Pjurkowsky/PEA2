#include <vector>
#include <string>
#include <iostream>
#include <limits.h>
#include <chrono>
#include "utils/NeighborhoodSolver.cpp"
#include "utils/RandomGenerator.h"


#ifndef GRAPH_H
#define GRAPH_H

class Graph
{
public:
    // constructor
    Graph(int numVertices);

    // destructor
    ~Graph();

    // method to add an edge between two vertices with a given weight
    void addEdge(int u, int v, int weight);

    // method to get the weight of an edge between two vertices
    int getEdgeWeight(int u, int v) const;

    // method to get the number of vertices in the graph
    int getNumVertices() const;

    // method to get the number of edges in the graph
    int getNumEdges() const;

    // method to get the stop time
    int getStopTime() const;

    // method to get the neighborhood choice
    int getNeighborhoodChoice() const;

    // method to get the initial temperature
    double getTemperature() const;

    // method to get the cooling rate
    double getAlpha() const;

    // method to set the stop time
    void setStopTime(int stopTime);
    
    // method to set the neighborhood choice
    void setNeighborhoodChoice(int neighborhoodChoice);

    // method to set the initial temperature
    void setTemperature(double temperature);

    // method to set the cooling rate
    void setAlpha(double alpha);
    
    // method to convert the graph into a string
    std::string toString() const;

    // method to get the adjacency matrix
    std::vector<std::vector<int>> getAdjecencyMatrix() const;

    // method to get cost of a path
    int getPathCost(const std::vector<int> &path) const;

    // method to get path by using greedy algorithm (nearest neighbour)
    void getGreedyPath(std::vector<int> &path, int &totalWeight, int startVertex);

    // method to optimize path by using tabu search algorithm
    void tabuSearch(std::vector<int> &path, int &totalWeight, int startVertex);

    // method to optimize path by using simulated annealing algorithm
    void simulatedAnnealing(std::vector<int> &path, int &totalWeight, int startVertex);

private:
    // store the weights of edges between vertices
    std::vector<std::vector<int>> adjacencyMatrix;

    // stop time for algorithms in seconds
    int stopTime;

    // neighborhood choice
    int neighborhoodChoice = 1;

    // cooling rate
    double alpha = 0.999; 
    
    // initial temperature
    double temperature = 1000; 

    // number of vertices
    int numVertices;

    // variable to store greedy path
    std::vector<int> greedyPath;

    // number of edges
    int numEdges;
};

#endif