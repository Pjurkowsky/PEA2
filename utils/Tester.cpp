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
    std::ofstream file(fileName, std::ios_base::app);
    if (!file)
        std::cout << "Unable to open file" << '\n';
    else
    { // find index of minimum total weight
        int minTotalWeightIndex = 0;
        for (int i = 0; i < totalWeights.size(); i++)
            if (totalWeights[i] < totalWeights[minTotalWeightIndex])
                minTotalWeightIndex = i;
        // print total weights and times to file
        for (int i = 0; i < totalWeights.size(); i++)
            file << totalWeights[i] << " " << times[i] << '\n';
        // print best total weight to file
        file << totalWeights[minTotalWeightIndex] << " ";
        // print best time to file
        file << times[minTotalWeightIndex] << " ";
        // print best path to file
        for (int i = 0; i < paths[minTotalWeightIndex].size(); i++)
            file << paths[minTotalWeightIndex][i] << " ";
    }

    file.close();
}

Graph *Tester::loadGraphFromFile(std::string fileName)
{
    std::ifstream file(fileName);
    if (!file)
    {
        std::cout << "Unable to open file" << '\n';
    }

    int numVertices;
    file >> numVertices;
    Graph *graph = new Graph(numVertices);
    int weight;
    for (int i = 0; i < numVertices; i++)
    {
        for (int j = 0; j < numVertices; j++)
        {
            file >> weight;
            graph->addEdge(i, j, weight);
        }
    }
    return graph;
}

void Tester::testTabuSearch(std::string fileName, int stopTime, int neighborhoodChoice)
{

    Graph *graph = loadGraphFromFile(fileName);
    graph->setStopTime(stopTime);
    graph->setNeighborhoodChoice(neighborhoodChoice);

    std::cout << "Testing tabu search algorithm..." << '\n';
    for (int i = 0; i < numOfTests; i++)
    {
        std::ofstream file(fileName + ".txt", std::ios_base::app);
        std::cout << "Test " << i + 1 << '\n';
        std::vector<int> path;
        int totalWeight = 0;
        int startVertex = 0;
        long time = 0;
        graph->tabuSearch(path, totalWeight, startVertex, time);

        file << totalWeight << " " << time << '\n';

        paths.push_back(path);
        totalWeights.push_back(totalWeight);
        times.push_back(time);
        file.close();
    }
    std::ofstream file(fileName + ".txt", std::ios_base::app);
    int minTotalWeightIndex = 0;
    for (int i = 0; i < totalWeights.size(); i++)
        if (totalWeights[i] < totalWeights[minTotalWeightIndex])
            minTotalWeightIndex = i;
    file << totalWeights[minTotalWeightIndex] << " " << times[minTotalWeightIndex] << " ";
    for (int i = 0; i < paths[minTotalWeightIndex].size(); i++)
        file << paths[minTotalWeightIndex][i] << " ";
    file << '\n';

    file.close();
}

void Tester::testSimulatedAnnealing(std::string fileName, int stopTime, int neighborhoodChoice)
{
    Graph *graph = loadGraphFromFile(fileName);
    graph->setStopTime(stopTime);
    graph->setNeighborhoodChoice(neighborhoodChoice);

    std::cout << "Testing simulated annealing algorithm..." << '\n';
    for (int i = 0; i < numOfTests; i++)
    {
        std::ofstream file(fileName + "sa.txt", std::ios_base::app);
        std::cout << "Test " << i + 1 << '\n';
        std::vector<int> path;
        int totalWeight = 0;
        int startVertex = 0;
        long time = 0;
        graph->simulatedAnnealing(path, totalWeight, startVertex, time);

        file << totalWeight << " " << time << '\n';

        paths.push_back(path);
        totalWeights.push_back(totalWeight);
        times.push_back(time);
        file.close();
    }
    std::ofstream file(fileName + "sa.txt", std::ios_base::app);
    int minTotalWeightIndex = 0;
    for (int i = 0; i < totalWeights.size(); i++)
        if (totalWeights[i] < totalWeights[minTotalWeightIndex])
            minTotalWeightIndex = i;
    file << totalWeights[minTotalWeightIndex] << " " << times[minTotalWeightIndex] << " ";
    for (int i = 0; i < paths[minTotalWeightIndex].size(); i++)
        file << paths[minTotalWeightIndex][i] << " ";
    file << '\n';

    file.close();

    paths.clear();
    totalWeights.clear();
    times.clear();
    
}
