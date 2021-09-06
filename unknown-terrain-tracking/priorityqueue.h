#include "node.h"
#include <algorithm>

class PriorityQueue
{
private:
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> p;

public:
    PriorityQueue() {}
    ~PriorityQueue() {}
    bool empty()
    {
        return p.size() == 0;
    }
    int size()
    {
        return p.size();
    }

    void put(Node &n, Key &priority)
    {
        n.priority = priority;
        p.push(n);
    }

    Node top()
    {
        return p.top();
    }

    Node pop()
    {
        Node n = p.top();
        p.pop();
        return n;
    }

    Key first_key()
    {
        return p.top().priority;
    }

    void deleteNode(const Node &n)
    {
        std::vector<Node> v;
        while (!p.empty())
        {
            Node nn = p.top();
            if (!(nn.x == n.x and nn.y == n.y))
                v.push_back(nn);
            p.pop();
        }
        for (Node &i : v)
            p.push(i);
    }

    bool isExist(const Node &n)
    {
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> temp = p;
        while (!temp.empty())
        {
            Node nn = temp.top();
            if (nn.x == n.x and nn.y == n.y)
                return true;
            temp.pop();
        }
        return false;
    }
};