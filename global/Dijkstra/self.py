#!/usr/bin/env python3
import numpy as np
import heapq

def main():
    graph = {
            'S':{'A':5, 'B':4, 'C':1},
            'A':{'S':5, 'D':2},
            'B':{'S':4, 'C':2, 'D':5, 'E':6},
            'C':{'S':1, 'B':2},
            'D':{'A':2, 'B':5, 'F':1, 'G':3},
            'E':{'B':6, 'G':2},
            'F':{'D':1, 'G':4},
            'G':{'D':3, 'E':2, 'F':4},
            }
    node_cost = {
            'S':{'value':0,      'defined':False},
            'A':{'value':np.inf, 'defined':False},
            'B':{'value':np.inf, 'defined':False},
            'C':{'value':np.inf, 'defined':False},
            'D':{'value':np.inf, 'defined':False},
            'E':{'value':np.inf, 'defined':False},
            'F':{'value':np.inf, 'defined':False},
            'G':{'value':np.inf, 'defined':False},
            }
    prev_node = {
            'S':None,
            'A':None,
            'B':None,
            'C':None,
            'D':None,
            'E':None,
            'F':None,
            'G':None,
            }


    for _ in range(10):
        # search the minimum cost node in undefined nodes
        min_node = None
        for key, val in node_cost.items():
            if val['defined'] == False:
                if min_node == None:
                    min_node = key
                elif val['value'] < node_cost[min_node]['value']:
                    min_node = key

        # When all nodes' cost are defined
        if min_node == None:
            print('Finish')
            break
        
        # define the minimum cost
        node_cost[min_node]['defined'] = True

        # calculate the cost of derived node from minimum cost node
        for node in graph[min_node]:
            if node_cost[min_node]['value']+graph[min_node][node] < node_cost[node]['value']:
                node_cost[node]['value'] = node_cost[min_node]['value']+graph[min_node][node]
                prev_node[node] = min_node

        # check
        for key, val in node_cost.items():
            print(key, val)
        print('*'*50)

    
    # Answer
    route = ['G']
    while True:
        route.insert(0,prev_node[route[0]])
        if route[0] == 'S':
            break

    print(route)



if __name__ == '__main__':
    main()
