import random
import itertools


with open(r"D:\workspace\c++\ComparativeTest\map2.txt", "r") as f:
    map = f.readlines()

for i in range(0, len(map)):
    map[i] = map[i].strip("\n")


num = 9

random_list = list((itertools.product(range(0, 20), range(0, 20))))
random_n_list = []
for i in range(0, len(random_list)):
    if map[random_list[i][0]][random_list[i][1]] == ".":
        random_n_list.append(random_list[i])
select_goal_node = random.sample(random_n_list, num)
print("Selected Start: ", end=" ")
for node in select_goal_node:
    print("{"+str(node[1])+","+str(node[0])+"},", end=" ")
print("{0,0}")


random_list = list((itertools.product(range(30, 60), range(30, 60))))
random_n_list = []
for i in range(0, len(random_list)):
    if map[random_list[i][0]][random_list[i][1]] == ".":
        random_n_list.append(random_list[i])
select_goal_node = random.sample(random_n_list, num)
print("Selected Goal: ", end=" ")
for node in select_goal_node:
    print("{"+str(node[1])+","+str(node[0])+"},", end=" ")
print("{59,59}")


'''
map2 配置1
Selected Start:  {5,8}, {8,14}, {5,16}, {13,12}, {14,14}, {12,2}, {2,10}, {0,2}, {7,6}, {0,0}
Selected Goal:  {30,35}, {53,56}, {39,52}, {49,34}, {35,42}, {42,41}, {38,44}, {54,46}, {44,58}, {59,59}

map2 配置2
Selected Start:  {12,18}, {6,16}, {1,16}, {2,3}, {6,10}, {2,4}, {12,9}, {15,12}, {11,0}, {0,0}
Selected Goal:  {54,37}, {36,50}, {53,30}, {31,50}, {36,32}, {32,42}, {57,40}, {48,51}, {45,46}, {59,59}
'''
