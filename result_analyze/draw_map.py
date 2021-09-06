import matplotlib.pyplot as plt
map_file_path = r"D:\workspace\c++\ComparativeTest\map3.txt"
moving_file_path = r"D:\workspace\c++\ComparativeTest\DstarLiteV4\result3_stop_map3\19_dsl_16_19_60_moving_path.txt"
goal_file_path = r"D:\workspace\c++\ComparativeTest\DstarLiteV4\result3_stop_map3\19_dsl_16_19_60_goal_path.txt"

save_name = "19_dsl_16_19_60_map3"

with open(map_file_path, "r") as f:
    map_grid = f.readlines()

with open(moving_file_path, 'r') as move_path_f:
    moving_path = move_path_f.readlines()

with open(goal_file_path, 'r') as goal_path_f:
    goal_path = goal_path_f.readlines()

for i in range(0, len(moving_path)):
    moving_path[i] = moving_path[i].strip("\n")

for i in range(0, len(goal_path)):
    goal_path[i] = goal_path[i].strip("\n")

for i in range(0, len(map_grid)):
    map_grid[i] = map_grid[i].strip("\n")

obstacle_x = []
obstacle_y = []
moving_path_x = []
moving_path_y = []
target_path_x = []
target_path_y = []

height = len(map_grid)
width = len(map_grid[0])
for i in range(height):
    for j in range(width):
        if map_grid[i][j] == "#":  # 栅格地图上obstacle为障碍物标识
            obstacle_y.append(height - 1 - i)
            obstacle_x.append(j)

for line in moving_path:
    moving_path_x.append(int(line.split(" ")[0]))
    moving_path_y.append(height - 1 - int(line.split(" ")[1]))

for line in goal_path:
    target_path_x.append(int(line.split(" ")[0]))
    target_path_y.append(height - 1 - int(line.split(" ")[1]))

plt.figure(figsize=(4,4))  # 为了防止x,y轴间隔不一样长，影响最后的表现效果，所以手动设定等长
plt.xlim(-0.5, height - 0.5)
plt.ylim(-0.5, width - 0.5)
plt.scatter(obstacle_x, obstacle_y, s=100, c='k', marker='s')

plt.scatter(target_path_x, target_path_y, s=100,
            c='green', alpha=0.8, marker='s')
plt.scatter(moving_path_x, moving_path_y, s=100,
            c='gold', alpha=0.8, marker='o')

plt.savefig(
    "D://workspace//c++//ComparativeTest//result_analyze//" + save_name + ".png")
plt.show()
