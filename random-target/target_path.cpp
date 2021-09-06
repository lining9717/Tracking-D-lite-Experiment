#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <random>
#include <memory>

using namespace std;

#define m_p(a, b) make_pair(a, b)

mt19937 rd(2016);

class TargetGenerator
{
private:
    vector<vector<int>> graph;
    int map_size, start, goal;
    int m = 0;
    int n = 0;
    vector<string> lab;
    vector<pair<int, int>> neigh = {m_p(0, 1), m_p(0, -1), m_p(1, 0), m_p(-1, 0)};
    set<int> visited;

    int getIndex(int x, int y)
    {
        return (x * m + y);
    }

    char getSell(int ind)
    {
        return lab[ind / m][ind % m];
    }

    int getNextGoal(int goal)
    {

        visited.insert(goal);
        if (goal - 1 > 0 and goal - 1 < map_size and (goal % m != 0) and
            getSell(goal - 1) == '.' and !visited.count(goal - 1) == 1)
        {
            return goal - 1;
        }
        else if (goal - n > 0 and goal - n < map_size and getSell(goal - n) == '.' and !visited.count(goal - n) == 1)
        {
            return goal - n;
        }
        else if (goal + 1 > 0 and goal + 1 < map_size and (goal % m != m - 1) and
                 getSell(goal + 1) == '.' and !visited.count(goal + 1) == 1)
        {
            return goal + 1;
        }
        else if (goal + n > 0 and goal + n < map_size and getSell(goal + n) == '.' and !visited.count(goal + n) == 1)
        {
            return goal + n;
        }
        return graph[goal][rd() % graph[goal].size()];
    }

public:
    TargetGenerator(const char *name)
    {
        ifstream myfile(name);
        string input;

        while (getline(myfile, input))
        {
            lab.push_back(input);
            m = max(m, (int)input.size());
        }
        int i_new, j_new;
        n = lab.size();
        graph.resize(n * m);
        map_size = n * m;
        start = 0;
        // cout << "Height: " << n << ", Width:" << m << endl;
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < m; ++j)
            {
                if (lab[i][j] != '.')
                    continue;
                for (auto it : neigh)
                {
                    i_new = i + it.first;
                    j_new = j + it.second;
                    if (i_new > -1 and i_new < n and j_new > -1 and j_new < m)
                        if (lab[i_new][j_new] == '.')
                            graph[getIndex(i, j)].push_back(getIndex(i_new, j_new));
                }
            }
        }
    }
    ~TargetGenerator() {}

    bool generateTargets(int gx, int gy, int number, string file_path)
    {
        if (gx < 0 or gx >= m or gy < 0 or gy >= n or lab[gy][gx] != '.')
        {
            cerr << "Goal(" << gx << ", " << gy << ") out of map bound." << endl;
            cout << "Input goal's x in [0, " << m - 1 << "], goal's y in [0, " << n - 1 << "]." << endl;
            return false;
        }
        goal = getIndex(gy, gx);
        fstream out_file(file_path, ios::out);
        if (!out_file.is_open())
        {
            cerr << "create out file fail!" << endl;
            return false;
        }

        int i = 0;
        while (i < number)
        {
            out_file << goal % m << " " << goal / m << endl;
            // cout << "{" << goal % m << ", " << goal / m << "},";
            goal = getNextGoal(goal);
            ++i;
        }
        // cout << endl;
        out_file.close();
        return true;
    }
};

int main(int argc, char const *argv[])
{
    const char *map_file = R"(D:\workspace\c++\ComparativeTest\map2.txt)";
    string out_file = R"(D:\workspace\c++\ComparativeTest\random-target\map2_point2\)";

    vector<pair<int, int>> goal = {
        {54, 37}, {36, 50}, {53, 30}, {31, 50}, {36, 32}, {32, 42}, {57, 40}, {48, 51}, {45, 46}, {59, 59}};
    // string out_name;
    int num = 100;
    int index = 0;

    while (index < goal.size())
    {
        // cout << "Please input goal(x, y): ";
        // cin >> gx >> gy;
        // cout << "Please input file_name: ";
        // cin >> out_name;
        // cout << "Please input number of targets: ";
        // cin >> num;
        cout << "goal (" << goal[index].first << "," << goal[index].second << ")" << std::endl;
        shared_ptr<TargetGenerator> p_tg = make_shared<TargetGenerator>(map_file);
        p_tg->generateTargets(goal[index].first, goal[index].second, num,
                              out_file + to_string(goal[index].first) + "_" + to_string(goal[index].second) + "_" + to_string(num) + ".txt");
        ++index;
    }
    return 0;
}
