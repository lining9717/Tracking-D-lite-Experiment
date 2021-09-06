#ifndef __D_STAR_LITE_H_
#define __D_STAR_LITE_H_

#include "node.h"
#include "squaredgrid.h"

class AgentViewGrid : public SquareGrid
{
public:
    AgentViewGrid() {}
    ~AgentViewGrid() {}
    AgentViewGrid(int w, int h) : SquareGrid(w, h)
    {
        width = w;
        height = h;
        x_bias = width / 2;
        y_bias = height / 2;
        // printf("AgentViewGrid set bias x:%d y:%d\n", x_bias, y_bias);
    }
    void new_walls(std::vector<std::pair<Node, char>> &observation, std::vector<Node> &results)
    {
        for (std::vector<std::pair<Node, char>>::iterator iter = observation.begin(); iter != observation.end(); ++iter)
        {
            Node n = iter->first;
            if (iter->second == WALL and !is_wall(n))
            {
                results.push_back(n);
            }
        }
    }

    void update_walls(std::vector<Node> &new_walls)
    {
        for (Node n : new_walls)
        {
            walls.insert(m_p(n.x, n.y));
        }
    }
};

#endif