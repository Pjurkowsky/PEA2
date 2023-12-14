#ifndef MENUITEM_H
#define MENUITEM_H

#include <iostream>
#include <vector>

struct MenuItem
{
    std::string str;
    std::vector<MenuItem> menuItems;
};

#endif