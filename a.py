# mathmodel21_v1.py
# Demo21 of mathematical modeling algorithm
# Demo of network flow problem optimization with NetworkX
# Copyright 2021 YouCans, XUPT
# Crated：2021-07-18

import numpy as np
import matplotlib.pyplot as plt  # 导入 Matplotlib 工具包
import networkx as nx  # 导入 NetworkX 工具包

# # 4. 多源多汇最小费用流问题 (Capacity network with multi source and multi sink)
# # 4.1 费用网络
# 创建有向图
G1 = nx.DiGraph()  # 创建一个有向图 DiGraph
G1.add_edges_from([('F1','W1',{'capacity': 9e5, 'weight': 2}), # F1~F2：工厂
                  ('F1','W2',{'capacity': 9e5, 'weight': 3}),  # W1~W2：仓库
                  ('F2','W1',{'capacity': 9e5, 'weight': 3}),
                  ('F2','W2',{'capacity': 9e5, 'weight': 1}),
                  ('W1','R1',{'capacity': 9e5, 'weight': 2}),  # R1~R4：零售店
                  ('W1','R2',{'capacity': 9e5, 'weight': 6}),
                  ('W1','R3',{'capacity': 9e5, 'weight': 3}),
                  ('W1','R4',{'capacity': 9e5, 'weight': 6}),
                  ('W2','R1',{'capacity': 9e5, 'weight': 4}),
                  ('W2','R2',{'capacity': 9e5, 'weight': 4}),
                  ('W2','R3',{'capacity': 9e5, 'weight': 6}),
                  ('W2','R4',{'capacity': 9e5, 'weight': 5})]) # 添加边的属性 'capacity', 'weight'
G1.add_node("F1", demand=-600)  # nx.min_cost_flow() 的设置要求
G1.add_node("F2", demand=-400)  # 设置源点的流量，负值表示净流出
G1.add_node("R1", demand=200)  # 设置汇点的流量，正值表示净流入
G1.add_node("R2", demand=150)
G1.add_node("R3", demand=350)
G1.add_node("R4", demand=300)
pos={'F1':(2,6.5),'F2':(2,3.5),'W1':(5,6.5),'W2':(5,3.5),'R1':(9,8),'R2':(9,6),'R3':(9,4),'R4':(9,2)}  # 指定顶点绘图位置

# # 4.2 用 NetworkX 求最小费用流
# 求最小费用流(demand=v)
minFlowCost = nx.min_cost_flow_cost(G1)  # 求最小费用流的费用
minFlowDict = nx.min_cost_flow(G1)  # 求最小费用流
# minFlowCost, minFlowDict = nx.network_simplex(G2)  # 求最小费用流--与上行等效

# 整理边的标签，用于绘图显示
# 数据格式转换
edgeCapacity = nx.get_edge_attributes(G1, 'weight')
edgeLabel = {}  # 边的标签
for i in edgeCapacity.keys():  # 整理边的标签，用于绘图显示
    edgeLabel[i] = f'w={edgeCapacity[i]:}'  # 边的容量
edgeLists = []
for i in minFlowDict.keys():
    for j in minFlowDict[i].keys():
        edgeLabel[(i,j)] += ',f=' + str(minFlowDict[i][j])  # 取出每条边流量信息存入边显示值
        if minFlowDict[i][j] > 0:
            edgeLists.append((i, j))

print("1. NetworkX 网络与图（最小费用流优化结果）：")  # NetworkX 工具包
print("最小费用:{}".format(minFlowCost))  # 输出最小费用的值
print("最小费用流的路径及流量: ", minFlowDict)  # 输出最小费用流的途径和各路径上的流量
print("最小费用流的路径：", edgeLists)  # 输出最小费用流的途径

# 绘制有向网络图
fig, ax = plt.subplots(figsize=(8,6))
ax.text(1.2,6.4,"600",color='navy')
ax.text(1.2,3.4,"400",color='navy')
ax.text(9.3,7.9,"200",color='navy')
ax.text(9.3,5.9,"150",color='navy')
ax.text(9.3,3.9,"350",color='navy')
ax.text(9.3,1.9,"300",color='navy')
ax.set_title("Capacity network with multi source and multi sink")
nx.draw(G1,pos,with_labels=True,node_color='skyblue',node_size=300,font_size=10)   # 绘制有向图，显示顶点标签
edgeLabel1 = nx.get_edge_attributes(G1, 'weight')
nx.draw_networkx_nodes(G1, pos, nodelist=['F1','F2'], node_color='orange')  # 设置指定顶点的颜色、宽度
nx.draw_networkx_nodes(G1, pos, nodelist=['W1','W2'], node_color='c')  # 设置指定顶点的颜色、宽度
nx.draw_networkx_edge_labels(G1,pos,edgeLabel,font_size=10,label_pos=0.25)  # 显示边的标签：'capacity','weight' + minCostFlow
nx.draw_networkx_edges(G1,pos,edgelist=edgeLists,edge_color='m',width=2)  # 设置指定边的颜色、宽度
plt.xlim(0.5, 10.5)
plt.ylim(1, 9)
plt.axis('on')
plt.show()