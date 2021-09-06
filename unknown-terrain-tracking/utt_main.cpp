#include "agentviewgrid.h"
#include "priorityqueue.h"

#include <sstream>
#include <chrono>

std::string target_name = "view4_map2";
const char *map_filepath = R"(D:\workspace\c++\ComparativeTest\map2.txt)";
const std::string target_goals_filepath = R"(D:\workspace\c++\ComparativeTest\random-target\map2_point1\)";
const std::string out_file = R"(D:\workspace\c++\ComparativeTest\unknown-terrain-tracking\map2_view4_result1_1-26\)";
const int view_range = 4;

bool is_get_target = true;
int num_of_target_positions = 100;

//配置一
std::vector<std::pair<int, int>> starts_vec = {{5, 8}, {8, 14}, {5, 16}, {13, 12}, {14, 14}, {12, 2}, {2, 10}, {0, 2}, {7, 6}, {0, 0}};
std::vector<std::pair<int, int>> goals_vec = {
    {30, 35}, {53, 56}, {39, 52}, {49, 34}, {35, 42}, {42, 41}, {38, 44}, {54, 46}, {44, 58}, {59, 59}};

//配置二
// std::vector<std::pair<int, int>> starts_vec = {{12, 18}, {6, 16}, {1, 16}, {2, 3}, {6, 10}, {2, 4}, {12, 9}, {15, 12}, {11, 0}, {0, 0}};
// std::vector<std::pair<int, int>> goals_vec = {
//     {54, 37}, {36, 50}, {53, 30}, {31, 50}, {36, 32}, {32, 42}, {57, 40}, {48, 51}, {45, 46}, {59, 59}};

int num_of_result = 1;

std::vector<std::pair<int, int>> goals;
int goal_index = 0;
int expansions_num = 0;

std::fstream out_result(out_file + "utt_" + target_name + "_result.txt", std::ios::out);

class UTT
{
private:
    AgentViewGrid grid;
    SquareGrid real_grid;
    bool is_show = false;
    int view_range = 2;
    int km = 0;
    Node position;
    Node goal;
    Node start;
    Node last_node;
    std::vector<int> G_VALS;
    std::vector<int> RHS_VALS;
    PriorityQueue frontier;
    std::vector<Node> new_walls;
    std::vector<std::pair<Node, char>> observation;
    int width = 0, height = 0;
    int map_size = 0;
    bool first_time = true;
    std::vector<int> parent;
    std::vector<Node> Delete_Nodes;

    std::fstream out_target_result;

public:
    UTT()
    {
        out_target_result.open(out_file + std::to_string(num_of_result) + "_utt_" + target_name + "_goal_path.txt", std::ios::out);
        if (!out_target_result.is_open())
        {
            std::cerr << "create out goal result file fail!" << std::endl;
        }
    }
    ~UTT()
    {
        out_target_result.close();
    }
    bool init(std::pair<int, int> nstart, std::pair<int, int> ngoal, const char *map, int v_range)
    {
        std::ifstream myfile(map);
        std::string input;
        std::vector<std::string> lab;

        view_range = v_range;
        km = 0;
        while (getline(myfile, input))
        {
            lab.push_back(input);
            width = std::max(width, (int)input.size());
        }
        height = lab.size();
        map_size = height * width;
        parent.resize(height * width, -1);
        G_VALS.resize(height * width, INFI);
        RHS_VALS.resize(height * width, INFI);

        real_grid = SquareGrid(width, height);
        grid = AgentViewGrid(width, height);

        // int start_col = grid.getColFromCoordinate(nstart.first);
        // int start_row = grid.getRowFromCoordinate(nstart.second);
        // int goal_col = grid.getColFromCoordinate(ngoal.first);
        // int goal_row = grid.getRowFromCoordinate(ngoal.second);
        int start_col = nstart.first;
        int start_row = nstart.second;
        int goal_col = ngoal.first;
        int goal_row = ngoal.second;

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                if (lab[i][j] == WALL)
                    real_grid.walls.insert(m_p(j, i));
                if (j == start_col and i == start_row)
                {
                    position = Node(j, i);
                    start = Node(j, i);
                }
                if (j == goal_col and i == goal_row)
                {
                    goal = Node(j, i);
                }
            }
        }
        return true;
    }

    inline int getIndex(const Node &n)
    {
        return n.y * width + n.x;
    }

    void setStart(std::pair<int, int> n)
    {
        position = Node(n.first, n.second);
        start = Node(n.first, n.second);
    }

    void setGoal(std::pair<int, int> n)
    {
        goal = Node(n.first, n.second);
    }

    void setIsShow(bool value)
    {
        is_show = value;
    }

    int heuristic(Node &a, Node &b)
    {
        return abs(a.x - b.x) + abs(a.y - b.y);
    }

    Key calculate_key(Node &n)
    {
        int g_rhs = std::min(g(n), rhs(n));
        return m_p(g_rhs + heuristic(n, position) + km, g_rhs);
    }

    bool is_key1_lowerthan_key2(Key k1, Key k2)
    {
        if (k1.first < k2.first or
            (k1.first == k2.first and k1.second < k2.second))
            return true;
        return false;
    }

    int rhs(const Node &n)
    {
        return RHS_VALS[getIndex(n)];
    }

    int g(const Node &n)
    {
        return G_VALS[getIndex(n)];
    }

    int different_nodes_number(std::deque<Node> &nodes)
    {
        std::deque<Node>::iterator it1;
        std::vector<std::pair<int, int>> v;
        for (it1 = nodes.begin(); it1 != nodes.end(); it1++)
            v.push_back(m_p(it1->x, it1->y));
        std::set<std::pair<int, int>> st(v.begin(), v.end());
        v.assign(st.begin(), st.end());
        int size = v.size();
        return size;
    }

    Node lowest_cost_neighbour(const Node &n)
    {
        std::vector<Node> neighbours;
        grid.neighbors(n, neighbours);
        Node lowerest_neighbour = neighbours[0];
        int min_cost = INFI;
        for (Node &neighbour : neighbours)
        {
            if (grid.is_wall(neighbour))
                continue;
            int cost = g(neighbour) + grid.cost(neighbour, n);
            if (min_cost > cost)
            {
                min_cost = cost;
                lowerest_neighbour = neighbour;
            }
        }
        return lowerest_neighbour;
    }

    int calculate_rhs(const Node &n)
    {
        std::vector<Node> neighbours;
        grid.neighbors(n, neighbours);
        Node lowerest_neighbour = neighbours[0];
        int min_cost = INFI;
        for (Node &neighbour : neighbours)
        {
            if (grid.is_wall(neighbour))
                continue;
            int cost = g(neighbour) + grid.cost(neighbour, n);
            if (min_cost > cost)
            {
                min_cost = cost;
                lowerest_neighbour = neighbour;
            }
        }
        parent[getIndex(n)] = getIndex(lowerest_neighbour);
        return min_cost;
    }

    void update_node(Node &n)
    {
        if (!isSameNodePosition(n, goal))
            RHS_VALS[getIndex(n)] = calculate_rhs(n);

        frontier.deleteNode(n);
        if (g(n) != rhs(n))
        {
            Key k = calculate_key(n);
            frontier.put(n, k);
            ++expansions_num;
        }
    }

    void update_nodes(std::vector<Node> &nodes)
    {
        int size = nodes.size();
        if (size == 0)
            return;
        for (int i = 0; i < size; ++i)
            update_node(nodes[i]);
    }

    void setParent(const Node &pnt, std::vector<Node> &children)
    {
        int pnt_index = getIndex(pnt);
        for (const Node &child : children)
        {
            if (!grid.is_wall(child))
                parent[getIndex(child)] = pnt_index;
        }
    }

    void dfsDelete(const Node &node)
    {
        frontier.deleteNode(node);
        RHS_VALS[getIndex(node)] = INFI;
        G_VALS[getIndex(node)] = INFI;

        parent[getIndex(node)] = -1;
        Delete_Nodes.emplace_back(node);
        std::vector<Node> neighs;
        grid.neighbors(node, neighs);
        for (const Node &nn : neighs)
            if (!isSameNodePosition(nn, goal) and
                parent[getIndex(nn)] == getIndex(node))
                dfsDelete(nn);
    }

    /**
     * @brief 使用BFS删除某一结点为根节点的搜索树，将树中每个结点冲frontier队列中删除，rhs、g值设为无穷大，父结点设为空，并加入Delete队列
     * 
     * @param node 所需删除搜索树的根节点
     */
    void bfsDelete(const Node &node)
    {
        std::queue<Node> q;
        q.push(node);
        std::vector<Node> neighs;
        while (!q.empty())
        {
            Node curr = q.front();
            frontier.deleteNode(curr);
            RHS_VALS[getIndex(curr)] = INFI;
            G_VALS[getIndex(curr)] = INFI;
            parent[getIndex(curr)] = -1;
            Delete_Nodes.emplace_back(curr);
            grid.neighbors(curr, neighs);
            for (const Node &nn : neighs)
            {
                if (!isSameNodePosition(nn, goal) and parent[getIndex(nn)] == getIndex(curr))
                    q.push(nn);
            }

            q.pop();
            neighs.clear();
        }
    }

    /**
     * @brief 判断两个结点位置是否相同
     * 
     * @param n1 
     * @param n2 
     * @return true 
     * @return false 
     */
    bool isSameNodePosition(const Node &n1, const Node &n2)
    {
        if (n1.x == n2.x and n1.y == n2.y)
            return true;
        return false;
    }

    /**
     * @brief 重用Delete中与当前已被扩展的结点相邻的结点，更新对应结点的rhs、parent、key，并加入frontier队列，最后删除Delete清空
     * 
     */
    void reuseDeletedNodes()
    {
        for (Node &dn : Delete_Nodes)
        {
            std::vector<Node> neighs;
            grid.neighbors(dn, neighs);
            for (const Node &nn : neighs)
            {
                if (grid.is_wall(dn))
                    continue;
                if (rhs(nn) == INFI or g(nn) == INFI)
                    continue;
                int temp_rhs = g(nn) + grid.cost(nn, dn);
                if (temp_rhs < rhs(dn))
                {
                    RHS_VALS[getIndex(dn)] = temp_rhs;
                    parent[getIndex(dn)] = getIndex(nn);
                    frontier.deleteNode(dn);
                    if (g(dn) != rhs(dn))
                    {
                        Key k = calculate_key(dn);
                        frontier.put(dn, k);
                        ++expansions_num;
                    }
                }
            }
        }
        Delete_Nodes.clear();
    }

    Node getNextGoal(const Node &goal)
    {
        if (goal_index >= goals.size())
            return goal;
        out_target_result << goal.x << " " << goal.y << std::endl;
        std::pair<int, int> temp = goals[goal_index++];
        return Node(temp.first, temp.second);
    }

    void draw_map(SquareGrid graph, std::vector<Node> &path)
    {
        bool isWrited = false;
        for (int i = 0; i < graph.height; ++i)
        {
            for (int j = 0; j < graph.width; ++j)
            {
                for (Node n : path)
                {
                    if (n.x == j and n.y == i and !isWrited and
                        !(j == start.x and i == start.y) and !(j == goal.x and i == goal.y))
                    {
                        std::cout << " " << PATH << " ";
                        isWrited = true;
                    }
                }
                if (!isWrited)
                {
                    if (graph.walls.count(m_p(j, i)))
                        std::cout << " " << WALL << " ";
                    else if (j == start.x and i == start.y)
                        std::cout << " " << START << " ";
                    else if (j == goal.x and i == goal.y)
                        std::cout << " " << GOAL << " ";
                    else
                        std::cout << " " << PASSABLE << " ";
                }
                isWrited = false;
            }
            std::cout << std::endl;
        }
    }

    void draw_map(SquareGrid graph, std::vector<int> &path)
    {
        bool isWrited = false;
        for (int i = 0; i < graph.height; ++i)
        {
            for (int j = 0; j < graph.width; ++j)
            {
                for (int n : path)
                {
                    if (n % width == j and n / width == i and !isWrited and
                        !(j == start.x and i == start.y) and !(j == goal.x and i == goal.y))
                    {
                        std::cout << " " << PATH << " ";
                        isWrited = true;
                    }
                }
                if (!isWrited)
                {
                    if (graph.walls.count(m_p(j, i)))
                        std::cout << " " << WALL << " ";
                    else if (j == start.x and i == start.y)
                        std::cout << " " << START << " ";
                    else if (j == goal.x and i == goal.y)
                        std::cout << " " << GOAL << " ";
                    else
                        std::cout << " " << PASSABLE << " ";
                }
                isWrited = false;
            }
            std::cout << std::endl;
        }
    }

    void draw_map(SquareGrid graph)
    {
        for (int i = 0; i < graph.height; ++i)
        {
            for (int j = 0; j < graph.width; ++j)
            {
                if (graph.walls.count(m_p(j, i)))
                    std::cout << " " << WALL << " ";
                else if (j == start.x and i == start.y)
                    std::cout << " " << START << " ";
                else if (j == goal.x and i == goal.y)
                    std::cout << " " << GOAL << " ";
                else
                    std::cout << " " << PASSABLE << " ";
            }
            std::cout << std::endl;
        }
    }

    void get_current_path(std::vector<int> &temp_path)
    {
        temp_path.clear();
        int pos_index = parent[getIndex(position)];
        while (pos_index != -1)
        {
            temp_path.emplace_back(pos_index);
            pos_index = parent[pos_index];
        }
    }

    int compute_shortest_path()
    {
        std::deque<Node> last_nodes;
        std::vector<Node> neighs;
        while (is_key1_lowerthan_key2(frontier.first_key(), calculate_key(position)) or g(position) != rhs(position))
        {
            if (frontier.size() == 0)
                return NO_PATH;
            Key k_old = frontier.first_key();
            Node node = frontier.pop();
            last_nodes.push_back(node);
            if (last_nodes.size() == 10 and different_nodes_number(last_nodes) < 3)
            {
                std::cout << "[UTT FAIL]:Stuck in a loop!" << std::endl;
                return NO_PATH;
            }

            Key k_new = calculate_key(node);

            if (is_key1_lowerthan_key2(k_old, k_new))
            {
                frontier.put(node, k_new);
                ++expansions_num;
            }

            else if (g(node) > rhs(node))
            {
                G_VALS[getIndex(node)] = RHS_VALS[getIndex(node)];
                grid.neighbors(node, neighs);
                update_nodes(neighs);
                neighs.clear();
            }
            else
            {
                G_VALS[getIndex(node)] = INFI;
                grid.neighbors(node, neighs);
                neighs.push_back(node);
                update_nodes(neighs);
                neighs.clear();
            }
        }
        return HAVE_PATH;
    }

    int move_to_goal(std::vector<Node> &path)
    {
        RHS_VALS[getIndex(goal)] = 0;
        Key goal_key = calculate_key(goal);
        frontier = PriorityQueue();
        frontier.put(goal, goal_key);
        ++expansions_num;

        real_grid.observe(position, view_range, observation);
        std::vector<Node> walls;
        grid.new_walls(observation, walls);
        grid.update_walls(walls);
        int status = compute_shortest_path();

        if (status == NO_PATH)
            return NO_PATH;
        Node last_position = position;
        Node last_goal = goal;
        path.push_back(position);
        std::vector<Node> new_walls;
        std::vector<int> curr_path;

        while (heuristic(goal, position) > 1)
        {
            if (g(position) == INFI)
            {
                std::cout << "No Path!!!" << std::endl;
                return NO_PATH;
            }
            position = lowest_cost_neighbour(position);
            start = position;
            km += heuristic(last_position, position);
            observation.clear();
            real_grid.observe(position, view_range, observation);
            grid.new_walls(observation, new_walls);

            last_goal = goal;
            if (goal_index < goals.size())
            {
                out_target_result << goal.x << " " << goal.y << std::endl;
                std::pair<int, int> temp = goals[goal_index++];
                goal = Node(temp.first, temp.second);
            }
            else
            {
                // std::cout << "Target get the goal" << std::endl;
                // return NO_PATH;
                is_get_target = false;
            }
            parent[getIndex(goal)] = -1;

            if (new_walls.size() or !isSameNodePosition(goal, last_goal))
            {
                if (!isSameNodePosition(goal, last_goal))
                {
                    bfsDelete(last_goal);
                    reuseDeletedNodes();
                }
                grid.update_walls(new_walls);

                last_position = position;
                for (const Node &wallnode : new_walls)
                {
                    std::vector<Node> neigh_nodes;
                    grid.neighbors(wallnode, neigh_nodes);
                    for (Node &node : neigh_nodes)
                        if (!grid.is_wall(node))
                            update_node(node);
                }
                status = compute_shortest_path();
                if (status == NO_PATH)
                    return NO_PATH;
            }

            path.push_back(position);
            new_walls.clear();
            if (is_show)
            {
                std::cout << "******************************************************" << std::endl;
                std::cout << "constructing graph(" << position.x << ", " << position.y << ")[A=Start, Z=Goal, @=path]:" << std::endl;
                get_current_path(curr_path);
                draw_map(grid, curr_path);
            }
        }
        // std::cout << "\n\n******************************************************" << std::endl;
        // std::cout << "Final Path, taken (@ symbols):" << std::endl;
        // draw_map(real_grid, path);
        return HAVE_PATH;
    }
};

int main(int argc, char const *argv[])
{

    if (!out_result.is_open())
    {
        std::cerr << "create out path result file fail!" << std::endl;
        return 1;
    }
    std::cout << "***********************[UTT]***************************" << std::endl;

    for (int i = 0; i < starts_vec.size(); ++i)
    {
        for (int j = 0; j < goals_vec.size(); ++j)
        {
            is_get_target = true;
            expansions_num = 0;
            int sx = starts_vec[i].first;
            int sy = starts_vec[i].second;
            int gx = goals_vec[j].first;
            int gy = goals_vec[j].second;
            std::cout << num_of_result << ": ";
            target_name = std::to_string(gx) + "_" + std::to_string(gy) + "_" + std::to_string(num_of_target_positions);
            std::cout << target_name << std::endl;

            std::ifstream targets_file(target_goals_filepath + target_name + ".txt");
            if (!targets_file.is_open())
            {
                std::cerr << "open targets_file file fail!" << std::endl;
                return 1;
            }
            std::fstream out_path_result(out_file + std::to_string(num_of_result) + "_utt_" + target_name + "_moving_path.txt", std::ios::out);
            if (!out_path_result.is_open())
            {
                std::cerr << "create out path result file fail!" << std::endl;
                return 1;
            }

            std::string position;
            while (getline(targets_file, position))
            {
                position.erase(std::remove(position.begin(), position.end(), '\n'), position.end());
                std::stringstream input(position);
                int x_str, y_str;
                input >> x_str;
                input >> y_str;
                goals.emplace_back(x_str, y_str);
            }
            goal_index = 1;
            gx = goals[0].first;
            gy = goals[0].second;
            std::cout << "Start: (" << sx << "," << sy << ")" << std::endl;
            std::cout << "Goal: (" << gx << "," << gy << ")" << std::endl;
            out_result << num_of_result << " (" << sx << "," << sy << ")"
                       << " (" << gx << "," << gy << ") ";

            std::chrono::system_clock::time_point start_time, end_time;
            start_time = std::chrono::system_clock::now();
            std::shared_ptr<UTT> ptr_utt = std::make_shared<UTT>();
            if (!ptr_utt->init(m_p(sx, sy), m_p(gx, gy), map_filepath, view_range))
            {
                std::cout << "init fail!" << std::endl;
                return 0;
            }

            std::vector<Node> path;
            ptr_utt->setIsShow(false);
            int status = ptr_utt->move_to_goal(path);
            end_time = std::chrono::system_clock::now();
            auto dura = (std::chrono::duration<double, std::milli>(end_time - start_time)).count();

            if (status == HAVE_PATH)
            {
                int length = path.size();
                std::cout << "spend time: " << dura << " ms" << std::endl;
                std::cout << "Path length: " << length << std::endl;
                out_result << dura << " " << length << " " << expansions_num;
                if (is_get_target)
                {
                    out_result << " Success" << std::endl;
                    std::cout << "State: Success" << std::endl;
                }
                else
                {
                    out_result << " Fail" << std::endl;
                    std::cout << "State: Fail" << std::endl;
                }

                for (int i = 0; i < length; i++)
                {
                    out_path_result << path[i].x << " " << path[i].y << std::endl;
                }
            }
            else
            {
                out_result << "-1 -1" << std::endl;
                std::cout << "No Path!" << std::endl;
            }
            out_path_result.close();
            goals.clear();
            ++num_of_result;
            std::cout << std::endl;
        }
    }
    out_result.close();
    return 0;
}