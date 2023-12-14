#include "Tester.h"

Tester::Tester(int numOfTests) : numOfTests(numOfTests)
{
}

//  method for generating graph
void Tester::generateGraph(Graph *graph)
{
    RandomGenerator random;
    int minWeight = 1;
    int maxWeight = random.generateRandomInt(10, 100);
    for (int i = 0; i < graph->getNumVertices(); i++)
        for (int j = 0; j < graph->getNumVertices(); j++)
            if (i != j)
                graph->addEdge(i, j, random.generateRandomInt(minWeight, maxWeight));
}

// method for printing results to file
void Tester::printResultsToFile(std::string fileName)
{
    std::ofstream file(fileName);
    if (!file)
        std::cout << "Unable to open file" << '\n';
    for (int i = 0; i < numbersOfVertices.size(); i++)
        file << numbersOfVertices[i] << " " << times[i] << '\n';
    file.close();
}

