#ifndef NEIGHBORHOODSOLVER_H
#define NEIGHBORHOODSOLVER_H

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include "RandomGenerator.h"

class NeighborhoodSolver
{
public:
    virtual std::vector<std::vector<int>> getNeighborhood(const std::vector<int> &path) = 0;
    virtual std::vector<int> solve(const std::vector<int> &path, int i, int j) = 0;
private:
    
};

class NeigborhoodSwapSolver : public NeighborhoodSolver
{
public:
    std::vector<std::vector<int>> getNeighborhood(const std::vector<int> &path)
    {
        std::vector<std::vector<int>> neighborhood;
        for (int i = 1; i < path.size() - 1; i++)
        {
            for (int j = 1; j < path.size() - 1; j++)
            {
                if (i != j)
                neighborhood.push_back(solve(path, i, j));
            }
        }
        return neighborhood;
    }

    std::vector<int> solve(const std::vector<int> &path, int i, int j) override
    {
        std::vector<int> newPath = path;
        std::swap(newPath[i], newPath[j]);
        return newPath;
    }
};

class NeigborhoodInsertSolver : public NeighborhoodSolver
{
public:
    std::vector<std::vector<int>> getNeighborhood(const std::vector<int> &path)
    {
        std::vector<std::vector<int>> neighborhood;
        for (int i = 1; i < path.size() - 1; i++)
        {
            for (int j = 1; j < path.size() - 1; j++)
            {
                if (i != j)
                    neighborhood.push_back(solve(path, i, j));
            }
        }
        return neighborhood;
    }

    std::vector<int> solve(const std::vector<int> &path, int i, int j) override
    {
        std::vector<int> newPath = path;
        int temp = newPath[i];
        newPath.erase(newPath.begin() + i);
        newPath.insert(newPath.begin() + j, temp);
        return newPath;
    }
};

class NeigborhoodInvertSolver : public NeighborhoodSolver
{
public:
    std::vector<std::vector<int>> getNeighborhood(const std::vector<int> &path)
    {
        std::vector<std::vector<int>> neighborhood;
        for (int i = 0; i < path.size(); i++)
        {
            for (int j = i + 1; j < path.size(); j++)
            {
                neighborhood.push_back(solve(path, i, j));
            }
        }
        return neighborhood;
    }
    std::vector<int> solve(const std::vector<int> &path, int i, int j) override
    {
        std::vector<int> newPath = path;
        std::reverse(newPath.begin() + i, newPath.begin() + j);
        return newPath;
    }
};

class NeigborhoodRandomInsertSolver : public NeighborhoodSolver
{
public:
   std::vector<std::vector<int>> getNeighborhood(const std::vector<int> &path)
    {
        std::vector<std::vector<int>> neighborhood;
        for (int i = 1; i < path.size() - 1; i++)
        {
            for (int j = 1; j < path.size() - 1; j++)
            {
                if (i != j)
                    neighborhood.push_back(solve(path, i, j));
            }
        }
        return neighborhood;
        
    }
    std::vector<int> solve(const std::vector<int> &path, int i, int j) override
    {   
        RandomGenerator random;
        std::vector<int> newPath = path;
        int temp = newPath[i];
        newPath.erase(newPath.begin() + i);
        newPath.insert(newPath.begin() + random.generateRandomInt(1, newPath.size() - 1), temp);
        return newPath;
    }
};

#endif