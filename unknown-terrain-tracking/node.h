#ifndef __NODE_H__
#define __NODE_H__

#include <algorithm>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <cmath>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <functional>

#define Key std::pair<int, int>
#define m_p(a, b) std::make_pair(a, b)
#define INFI 1e8 + 7
#define WALL '#'
#define PASSABLE '.'
#define PATH '@'
#define START 'A'
#define GOAL 'Z'
#define HAVE_PATH 0x123f
#define NO_PATH 0xf321
#define GET_GOAL 0xf322

struct Node
{
    int x;
    int y;
    Key priority;
    Node()
    {
        x = y = -1;
        priority = m_p(0, 0);
    }
    Node(int x, int y) : x(x), y(y)
    {
        priority = m_p(0, 0);
    }
    Node(int x, int y, Key p) : x(x), y(y), priority(p)
    {
    }
    bool operator<(const Node &g) const
    {
        if (priority.first < g.priority.first or
            (priority.first == g.priority.first and priority.second < g.priority.second))
            return true;
        return false;
    }
    bool operator>(const Node &g) const
    {
        if (priority.first < g.priority.first or
            (priority.first == g.priority.first and priority.second < g.priority.second))
            return false;
        return true;
    }

    bool operator!=(const Node &g) const
    {
        if (priority.first == g.priority.first and priority.second == g.priority.second)
            return false;
        return true;
    }

    bool operator==(const Node &g) const
    {
        if (priority.first == g.priority.first and priority.second == g.priority.second)
            return true;
        return false;
    }
};

#endif