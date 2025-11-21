import networkx

edges = [(1.1,1),(0,"A")]
edges = [(1.0,1.1),(1.1,1.2),(1.2,1.3),(1.0,2.0),(2.0,2.1),(2.1,2.2),(2.2,2.3),(2.0,2.4),(2.4,2.5),(2.0,3.0),(3.0,3.1)]
print(edges)
G = networkx.Graph()
G.add_edges_from(edges)

# 绘图
import matplotlib.pyplot as plt

pos = networkx.spring_layout(G)
networkx.draw(G, pos, with_labels=True, node_color='lightblue', 
              node_size=500, arrowsize=20)
plt.show()

a ={1:2,2:{2:1}}
print(a[2])