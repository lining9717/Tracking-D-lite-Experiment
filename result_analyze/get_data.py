
cost_time = []
cost_length = []
cost_expansion = []
cost_rating = []


result_path = r"D:\workspace\c++\ComparativeTest\unknown-terrain-tracking\map2_view4_result1_1-26\utt_view4_map2_result.txt"


with open(result_path, 'r') as f:
    data = f.readlines()
    for line in data:
        row_data = line.strip('\n').split(" ")
        if row_data[-1] == '-1':
            cost_time.append(-1)
            cost_length.append(-1)
            cost_expansion.append(-1)
            cost_rating.append(0)
            continue
        cost_time.append(float(row_data[3]))
        cost_length.append(int(row_data[4]))
        cost_expansion.append(int(row_data[5]))
        if row_data[6] == "Success":
            cost_rating.append(1)
        else:
            cost_rating.append(0)


for i in range(0,len(cost_time)):
    print(cost_time[i],cost_length[i],cost_expansion[i],cost_rating[i])
