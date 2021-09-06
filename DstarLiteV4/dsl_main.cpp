#include "agentviewgrid.h"
#include "priorityqueue.h"

#include <sstream>
#include <chrono>

std::string target_name = "view2_map2";
std::string map_filepath = R"(D:\workspace\c++\ComparativeTest\map2.txt)";
std::string target_goals_filepath = R"(D:\workspace\c++\ComparativeTest\random-target\map2_point1\)";
std::string out_file = R"(D:\workspace\c++\ComparativeTest\DstarLiteV4\map2_view4_result1\)";
int view_range = 4;

bool is_get_target = true;
int num_of_target_positions = 100;

int num_of_result = 1;
int expansions_num = 0;

//配置一
std::vector<std::pair<int, int>> starts_vec = {{5, 8}, {8, 14}, {5, 16}, {13, 12}, {14, 14}, {12, 2}, {2, 10}, {0, 2}, {7, 6}, {0, 0}};
std::vector<std::pair<int, int>> goals_vec = {
    {30, 35}, {53, 56}, {39, 52}, {49, 34}, {35, 42}, {42, 41}, {38, 44}, {54, 46}, {44, 58}, {59, 59}};

//配置二
// std::vector<std::pair<int, int>> starts_vec = {{12, 18}, {6, 16}, {1, 16}, {2, 3}, {6, 10}, {2, 4}, {12, 9}, {15, 12}, {11, 0}, {0, 0}};
// std::vector<std::pair<int, int>> goals_vec = {
//     {54, 37}, {36, 50}, {53, 30}, {31, 50}, {36, 32}, {32, 42}, {57, 40}, {48, 51}, {45, 46}, {59, 59}};
class DStarLite
{
private:
    AgentViewGrid grid;
    SquareGrid real_grid;
    bool is_show = false;
    int view_range = 2;
    std::vector<std::pair<Node, Node>> back_pointer;
    int km = 0;
    Node position;
    Node goal;
    Node start;
    std::map<std::pair<int, int>, int> G_VALS;
    std::map<std::pair<int, int>, int> RHS_VALS;
    PriorityQueue frontier;
    std::vector<Node> new_walls;
    std::vector<std::pair<Node, char>> observation;
    Node last_node;
    bool first_time = true;

public:
    DStarLite() {}
    ~DStarLite() {}
    bool init(std::pair<int, int> nstart, std::pair<int, int> ngoal, const std::string &map, int v_range)
    {
        std::ifstream myfile(map);
        std::string input;
        std::vector<std::string> lab;
        int width = 0, height = 0;

        view_range = v_range;
        km = 0;
        while (getline(myfile, input))
        {
            lab.push_back(input);
            width = std::max(width, (int)input.size());
        }
        height = lab.size();

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
                    goal = Node(j, i);
            }
        }
        if (start.x == -1 and start.y == -1 or goal.x == -1 and goal.y == -1)
        {
            std::cout << "!!!!NO START AND GOAL IN MAP!!!" << std::endl;
            return false;
        }

        Key goal_key = calculate_key(goal);
        frontier = PriorityQueue();
        frontier.put(goal, goal_key);
        ++expansions_num;
        back_pointer.push_back(m_p(goal, Node(-1, -1)));
        return true;
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

    int rhs(Node &n)
    {
        if (n.x == goal.x and n.y == goal.y)
            return 0;

        if (RHS_VALS.size() == 0 or RHS_VALS.count(m_p(n.x, n.y)) == 0)
            return INFI;
        return RHS_VALS[m_p(n.x, n.y)];
    }

    int g(Node &n)
    {
        if (G_VALS.size() == 0 or G_VALS.count(m_p(n.x, n.y)) == 0)
            return INFI;
        return G_VALS[m_p(n.x, n.y)];
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
        v.clear();
        v.shrink_to_fit();
        return size;
    }

    Node lowest_cost_neighbour(Node &n)
    {
        std::vector<Node> neighbours;
        grid.neighbors(n, neighbours);
        Node lowerest_neighbour = neighbours[0];
        int min_cost = INFI;
        for (Node neighbour : neighbours)
        {
            int cost = g(neighbour) + grid.cost(neighbour, n);
            if (min_cost > cost)
            {
                min_cost = cost;
                lowerest_neighbour = neighbour;
            }
        }
        neighbours.clear();
        neighbours.shrink_to_fit();
        return lowerest_neighbour;
    }

    int calculate_rhs(Node &n)
    {
        std::vector<Node> neighbours;
        grid.neighbors(n, neighbours);
        Node lowerest_neighbour = neighbours[0];
        int min_cost = INFI;
        for (Node neighbour : neighbours)
        {
            int cost = g(neighbour) + grid.cost(neighbour, n);
            if (min_cost > cost)
            {
                min_cost = cost;
                lowerest_neighbour = neighbour;
            }
        }
        back_pointer.push_back(m_p(n, lowerest_neighbour));
        neighbours.clear();
        neighbours.shrink_to_fit();
        return min_cost;
    }

    void update_node(Node &n)
    {
        if (!(n.x == goal.x and n.y == goal.y))
            RHS_VALS[m_p(n.x, n.y)] = calculate_rhs(n);
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
        for (int i = 0; i < size; i++)
            update_node(nodes[i]);
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
                std::cout << "[DStarLite FAIL]:Stuck in a loop!" << std::endl;
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
                G_VALS[m_p(node.x, node.y)] = RHS_VALS[m_p(node.x, node.y)];
                grid.neighbors(node, neighs);
                update_nodes(neighs);
                neighs.clear();
                neighs.shrink_to_fit();
            }
            else
            {
                G_VALS[m_p(node.x, node.y)] = INFI;
                grid.neighbors(node, neighs);
                neighs.push_back(node);
                update_nodes(neighs);
                neighs.clear();
                neighs.shrink_to_fit();
            }
        }
        return HAVE_PATH;
    }

    int move_to_goal(std::vector<Node> &path)
    {
        // std::cout << "The real graph (A=Start, Z=Goal):" << std::endl;
        // draw_map(real_grid);
        std::vector<std::pair<Node, char>> observation;
        real_grid.observe(position, view_range, observation);
        std::vector<Node> walls;
        grid.new_walls(observation, walls);
        grid.update_walls(walls);
        int status = compute_shortest_path();
        if (status == NO_PATH)
            return NO_PATH;

        Node last_node = position;
        path.push_back(position);
        std::vector<Node> new_walls;

        while (!(position.x == goal.x and position.y == goal.y))
        {
            if (is_show)
            {
                std::cout << "******************************************************" << std::endl;
                std::cout << "constructing graph (A=Start, Z=Goal, @=path):" << std::endl;
                draw_map(grid, path);
            }
            if (g(position) == INFI)
            {
                std::cout << "No Path!!!" << std::endl;
                return NO_PATH;
            }
            position = lowest_cost_neighbour(position);
            std::vector<std::pair<Node, char>>().swap(observation);
            real_grid.observe(position, view_range, observation);

            grid.new_walls(observation, new_walls);

            if (new_walls.size())
            {
                grid.update_walls(new_walls);
                km += heuristic(last_node, position);
                last_node = position;
                for (Node wallnode : new_walls)
                {
                    std::vector<Node> neigh_nodes;
                    grid.neighbors(wallnode, neigh_nodes);
                    for (Node node : neigh_nodes)
                        if (!grid.is_wall(node))
                            update_node(node);
                }
                status = compute_shortest_path();
                if (status == NO_PATH)
                    return NO_PATH;
            }
            path.push_back(position);
            std::vector<Node>().swap(new_walls);
        }
        // std::cout << "******************************************************" << std::endl;
        // std::cout << "Final Path, taken (@ symbols):" << std::endl;
        // draw_map(real_grid, path);
        return HAVE_PATH;
    }

    void draw_map_final(std::vector<Node> &path)
    {
        std::cout << "******************************************************" << std::endl;
        std::cout << "Final Path, taken (@ symbols):" << std::endl;
        draw_map(real_grid, path);
    }
};

int main(int argc, char *argv[])
{

    if (argc != 6)
    {
        std::cerr << "paramater error" << std::endl;
        std::cout << "[argv]: target_name, map_filepath, target_goals_filepath, out_file, view_ranger" << std::endl;
        return 1;
    }
    target_name = argv[1];
    map_filepath = argv[2];
    target_goals_filepath = argv[3];
    out_file = argv[4];
    view_range = atoi(argv[5]);

    std::cout << "***********************[D*Lite]***************************" << std::endl;
    std::fstream out_result(out_file + "dsl_" + target_name + "_result.txt", std::ios::out);
    if (!out_result.is_open())
    {
        std::cerr << "create out path result file fail!" << std::endl;
        return 1;
    }
    std::vector<std::pair<int, int>> goals;
    for (int i = 0; i < starts_vec.size(); ++i)
    {
        for (int j = 0; j < goals_vec.size(); ++j)
        {
            expansions_num = 0;
            int sx = starts_vec[i].first;
            int sy = starts_vec[i].second;
            int gx = goals_vec[j].first;
            int gy = goals_vec[j].second;
            std::cout << num_of_result << ": ";
            target_name = std::to_string(gx) + "_" + std::to_string(gy) + "_" + std::to_string(num_of_target_positions);
            std::cout << target_name << std::endl;

            std::ifstream targets_file(target_goals_filepath + target_name + ".txt");
            std::fstream out_path_result(out_file + std::to_string(num_of_result) + "_dsl_" + target_name + "_moving_path.txt", std::ios::out);
            std::fstream out_target_result(out_file + std::to_string(num_of_result) + "_dsl_" + target_name + "_goal_path.txt", std::ios::out);
            if (!targets_file.is_open())
            {
                std::cerr << "open targets_file file fail!" << std::endl;
                return 1;
            }
            if (!out_path_result.is_open())
            {
                std::cerr << "create out path result file fail!" << std::endl;
                return 1;
            }
            if (!out_target_result.is_open())
            {
                std::cerr << "create out goal result file fail!" << std::endl;
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
            int goal_index = 1;
            gx = goals[0].first;
            gy = goals[0].second;
            std::cout << "Start: (" << sx << "," << sy << ")" << std::endl;
            std::cout << "Goal: (" << gx << "," << gy << ")" << std::endl;
            out_result << num_of_result << " (" << sx << "," << sy << ")"
                       << " (" << gx << "," << gy << ") ";

            bool flag = true;
            is_get_target = true;

            std::chrono::system_clock::time_point start_time, end_time;
            start_time = std::chrono::system_clock::now();
            std::vector<Node> final_path;
            final_path.emplace_back(sx, sy);
            std::vector<Node> path;
            while (1)
            {
                path.clear();
                std::shared_ptr<DStarLite> ptr_dsl = std::make_shared<DStarLite>();
                if (!ptr_dsl->init(m_p(sx, sy), m_p(gx, gy), map_filepath, view_range))
                {
                    std::cout << "init fail!" << std::endl;
                    return 0;
                }
                ptr_dsl->setIsShow(false);
                int status = ptr_dsl->move_to_goal(path);
                if (path.size() <= 1)
                {
                    // ptr_dsl->draw_map_final(final_path);
                    flag = true;
                    break;
                }

                end_time = std::chrono::system_clock::now();
                auto dura = (std::chrono::duration<double, std::milli>(end_time - start_time)).count();
                if (dura >= 60000)
                {
                    flag = false;
                    break;
                }

                if (status == HAVE_PATH)
                {
                    sx = path[1].x;
                    sy = path[1].y;
                    final_path.emplace_back(sx, sy);
                    if (goal_index < goals.size())
                    {
                        out_target_result << gx << " " << gy << std::endl;
                        gx = goals[goal_index].first;
                        gy = goals[goal_index].second;
                        ++goal_index;
                    }
                    else
                    {
                        is_get_target = false;
                    }
                }
                else
                {
                    std::cout << "No Path!" << std::endl;
                    flag = false;
                    break;
                }
            }
            end_time = std::chrono::system_clock::now();
            auto dura = (std::chrono::duration<double, std::milli>(end_time - start_time)).count();
            int length = final_path.size();
            for (int i = 0; i < length; i++)
            {
                out_path_result << final_path[i].x << " " << final_path[i].y << std::endl;
            }
            if (flag)
            {
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
            }
            else
            {
                out_result << -1 << " " << -1 << std::endl;
            }

            flag = true;

            out_path_result.close();
            out_target_result.close();
            goals.clear();
            ++num_of_result;
            std::cout << std::endl;
        }
    }
    out_result.close();
    return 0;
}
