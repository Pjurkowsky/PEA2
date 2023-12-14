#ifndef RANDOMGENERATOR_H
#define RANDOMGENERATOR_H

#include <iostream>
#include <chrono>
#include <random>
#include <fstream>
#include <algorithm>

class RandomGenerator
{
public:
    RandomGenerator();                                                      // constructor
    int generateRandomInt(int min, int max) ;                                // generates random integer
    int *generateArrayOfInts(size_t size, int min, int max) ;                // generates array of random integers
    std::vector<std::vector<int>> shuffleMatrix(const std::vector<std::vector<int>> &matrix) ; // shuffles matrix
    void generateFile(std::string filename, int numbers, int min, int max) ; // generates file with random integers

private:
    std::random_device rd;
    std::mt19937 rng;
};

#endif