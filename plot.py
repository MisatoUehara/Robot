import random

# 原始列表
data = ['1.1','1.2','1.3','2.1','2.2','2.3','2.4','2.5','3.1']

# 随机选择N个元素，允许重复
N = 5  # 设置要选择的数量
result = random.choices(data, k=N)
result_dict = {}
for item in result:
    result_dict[item] = result_dict.get(item, 0) + 1
print(result)
print(result_dict)

random.seed(0)
D={i:random.randint(0,10) for i in range(10)}       #需求
d={}
d[i]
print(D)