#ifndef __D_STAR_LITE_SQUARED_GRID_H_
#define __D_STAR_LITE_SQUARED_GRID_H_

#include "node.h"

class SquareGrid
{
public:
    int width;
    int height;
    int x_bias;
    int y_bias;
    std::set<std::pair<int, int>> walls;
    std::vector<std::pair<int, int>> neigh = {m_p(0, 1), m_p(0, -1), m_p(1, 0), m_p(-1, 0)};

    SquareGrid() {}
    ~SquareGrid() {}
    SquareGrid(int w, int h)
    {
        width = w;
        height = h;
        x_bias = width / 2;
        y_bias = height / 2;
        // printf("SquareGrid set bias x:%d y:%d\n", x_bias, y_bias);
    }
    bool in_bound(int x, int y)
    {
        return 0 <= x and x < width and 0 <= y and y < height;
    }

    bool is_wall(Node &n)
    {
        for (std::set<std::pair<int, int>>::iterator it = walls.begin(); it != walls.end(); it++)
        {
            if ((*it).first == n.x and (*it).second == n.y)
                return true;
        }
        return false;
    }

    int cost(Node &from_node, Node &to_node)
    {
        if (is_wall(from_node) or is_wall(to_node))
            return INFI;
        return 1;
    }

    void neighbors(Node &n, std::vector<Node> &results)
    {
        for (auto it : neigh)
        {
            // std::cout << "x:" << n.x + it.first << ",y:" << n.y + it.second << std::endl;
            if (in_bound(n.x + it.first, n.y + it.second))
            {
                Node nn(n.x + it.first, n.y + it.second);
                results.push_back(nn);
            }
        }
    }

    void observe(Node &position, int obs_range, std::vector<std::pair<Node, char>> &results)
    {
        for (int x = position.x - obs_range;
             x <= position.x + obs_range; x++)
        {
            for (int y = position.y - obs_range;
                 y <= position.y + obs_range; y++)
            {

                if (in_bound(x, y))
                {
                    Node n(x, y);
                    if (is_wall(n))
                        results.push_back(std::pair<Node, char>(n, WALL));
                    else
                        results.push_back(std::pair<Node, char>(n, PASSABLE));
                }
            }
        }
    }
    void print_walls()
    {
        std::cout << "Walls:";
        for (std::pair<int, int> n : walls)
        {
            std::cout << "(" << n.first << "," << n.second << ") ";
        }
        std::cout << std::endl;
    }

    int getColFromCoordinate(int x)
    {
        if (x + x_bias < 0 or x + x_bias >= width)
            printf("[ERROR] X out of bound\n");
        return x + x_bias;
    }

    int getRowFromCoordinate(int y)
    {
        if (y_bias - y < 0 or y_bias - y >= height)
            printf("[ERROR] Y out of bound\n");
        return y_bias - y;
    }

    int getXFromCol(int col)
    {
        return col - x_bias;
    }

    int getYFromRow(int row)
    {
        return y_bias - row;
    }
};

#endif