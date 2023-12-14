#include <iostream>
#include "utils/Menu.h"
#include <string.h>
#include "Graph.h"

std::vector<MenuItem> displayOptions = {{"display matrix", {}},
                                        {"display list", {}},
                                        {"exit", {}}};

std::vector<MenuItem> tabuAlgorithmOptions = {{"set neighborhood", {}},
                                              {"set stop time", {}},
                                              {"run algorithm", {}},
                                              {"exit", {}}};

std::vector<MenuItem> saAlgorithmOptions = {{"set initial temperature", {}},
                                            {"set neighborhood", {}},
                                            {"set cooling rate", {}},
                                            {"set stop time", {}},
                                            {"run algorithm", {}},
                                            {"exit", {}}};

std::vector<MenuItem> mainMenuItems = {{"read from file", {}},
                                       {"generate graph", {}},
                                       {"display graph", {displayOptions}},
                                       {"create starting solution", {}},
                                       {"set stop time", {}},
                                       {"tabu search algorithm", {tabuAlgorithmOptions}},
                                       {"simulated annealing algorithm", {saAlgorithmOptions}},
                                       {"exit", {}}};

std::vector<MenuItem> testMenuItems = {{"exit", {}}};

int main(int argc, char const *argv[])
{

    if (argc > 1)
    {
        for (int i = 0; i < argc; i++)
            if (strcmp(argv[i], "-t") == 0)
            {
                Menu menu("Test Menu", testMenuItems, new Graph(0));
                menu.run();
            }
    }
    else
    {
        Menu menu("Main Menu", mainMenuItems, new Graph(0));
        menu.run();
    }

    return 0;
}
