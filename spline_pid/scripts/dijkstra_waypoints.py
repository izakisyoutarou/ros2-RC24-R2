import networkx as nx

# ファイルの読み込み
edgelist = nx.read_weighted_edgelist('config/edgelist.txt')

path = nx.dijkstra_path(edgelist, 'I', 'H')
length = nx.dijkstra_path_length(edgelist, 'I', 'H')

print(path)
print(length)
