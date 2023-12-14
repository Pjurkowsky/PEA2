#include "RandomGenerator.h"

RandomGenerator::RandomGenerator() : rng(rd())
{
    rng.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

std::vector<std::vector<int>> RandomGenerator::shuffleMatrix(const std::vector<std::vector<int>> &matrix) 
{
    std::vector<std::vector<int>> shuffledMatrix = matrix;
    std::shuffle(shuffledMatrix.begin(), shuffledMatrix.end(), rng);
    return shuffledMatrix;
}

// generates random integer
int RandomGenerator::generateRandomInt(int min, int max) 
{
    std::uniform_int_distribution<int> uint_dist(min, max);
    return uint_dist(rng);
}

// generates array of random integers
int *RandomGenerator::generateArrayOfInts(size_t size, int min, int max) 
{
    int *x = new int[size];

    for (int i = 0; i < size; i++)
    {
        int y = generateRandomInt(min, max);
        for (int j = 0; j < i; j++)
            while (x[j] == y)
                y = generateRandomInt(min, max);
        x[i] = y;
    }
    return x;
}

// generates file with random integers
void RandomGenerator::generateFile(std::string filename, int numbers, int min, int max) 
{
    std::ofstream file(filename);

    if (!file)
    {
        std::cout << "Unable to create file\n";
        return;
    }
    int *x = generateArrayOfInts(numbers, min, max);

    for (int i = 0; i < numbers; i++)
        file << x[i] << '\n';

    delete[] x;
    file.close();
}
