#ifndef TESTER_H
#define TESTER_H

#define INT_MAX __INT_MAX__

#include <iostream>
#include "Timer.h"
#include <fstream>
#include "../Graph.h"
#include "RandomGenerator.h"
class Tester
{
public:
    Tester(int numOfTests);                        // constructor
    void generateGraph(Graph *graph);              // method for generating graph
    void printResultsToFile(std::string fileName); // method for printing results to file

private:
    int numOfTests;
    std::vector<int> numbersOfVertices;
    std::vector<double> times;
};

#endif
