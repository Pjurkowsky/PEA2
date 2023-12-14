#ifndef MENU_H
#define MENU_H

#include <iostream>
#include <string>
#include <fstream>
#include <random>
#include <ctime>
#include <climits>
#include <functional>
#include <algorithm>
#include <chrono>
#include <iomanip>

#include "MenuItem.h"
#include "Timer.h"
#include "RandomGenerator.h"
#include "Tester.h"
#include "../Graph.h"
class Menu
{
public:
    Menu(std::string name, std::vector<MenuItem> menuItems, Graph *graph); // constructor
    bool run();                                                            // runs menu

private:
    void waitForUser();                              // waits for user to press enter
    int getIntInput(std::string message);            // gets int input from
    float getFloatInput(std::string message);        // gets float input from user
    std::string getStringInput(std::string message); // gets string input from user

    std::vector<MenuItem> menuItems;
    bool innerLoop;
    std::string menuName;
    Graph *graph;
};

#endif
