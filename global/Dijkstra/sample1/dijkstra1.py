#!/usr/bin/env python3
from heapq import heappush, heappop
import numpy as np
import sys
import re

"""
グラフ初期化
"""
def initialize_graph():
    graph = {}
    edge_list = read_file()
    for edge_inf in edge_list:
        node1 = edge_inf.split(',')[0]
        node2 = edge_inf.split(',')[1]
        cost  = int(edge_inf.split(',')[2])

        graph = set_new_node(graph, node1, node2)
        graph[node1][node2] = cost
        graph[node2][node1] = cost
    
    return graph

"""
グラフファイル読み込み
"""
def read_file():
    file_name = 'graph1.csv'
    edge_list = []
    with open(file_name, 'r', encoding='utf-8') as fin:
        for line in fin.readlines():
            edge_list.append(re.sub(r'[\r\n]','',line))

    return edge_list

"""
ノードをグラフに追加
"""
def set_new_node(graph, node1, node2):
    if not node1 in graph.keys():
        graph[node1] = {}
    if not node2 in graph.keys():
        graph[node2] = {}
    return graph

"""
ダイクストラ法で解く
"""
def dijkstra_solver(graph):
    #各ノードのSTARTからの最小コスト
    min_dist_dict = {}
    #最短経路における直前ノード
    prev_node_dict = {}
    q = []
    heappush(q, (0, 'START'))
    print(q)
    prev_node = ''

    for i in range(1):
        #qのうち最小コストのものを取り出す
        dist, node = heappop(q)
        #それをSTARTからの最短経路とする    
        min_dist_dict[node] = dist
        prev_node_dict[node] = prev_node

        if node == 'GOAL':
            return min_dist_dict, prev_node_dict
        prev_node = node

        calc_min_dist_and_put(graph, q, node, min_dist_dict, prev_node_dict)

def calc_min_dist_and_put(graph, q, departure_node, min_dist_dict, prev_node_dict):


    


def main():
    graph = initialize_graph()
    print(graph)
    dijkstra_solver(graph)

if __name__ == '__main__':
    main()
