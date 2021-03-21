#!/usr/bin/env python3

import numpy as np

#未確定ノード、各ノードの移動コストの初期値
node_l = {
        'S':0,
        'A':np.inf,
        'B':np.inf,
        'C':np.inf,
        'D':np.inf,
        'E':np.inf,
        'F':np.inf,
        'G':np.inf,
        }

#確定ノード
node_l_ = {}

#各エッジの移動コスト（固定値）
edge = {
        'S':{'A':5, 'B':4, 'C':1},
        'A':{'S':5, 'D':2},
        'B':{'S':4, 'C':2, 'D':5, 'E':6},
        'C':{'S':1, 'B':2},
        'D':{'A':2, 'B':5, 'F':1, 'G':3},
        'E':{'B':6, 'G':2},
        'F':{'D':1, 'G':3},
        'G':{'D':3, 'E':2, 'F':4},
        }

prev_node = {}

#while True:
for _ in range(10):
    """
    ノードコストが最も低いものをmin_nodeとする
    """
    if len(node_l) == 1:
        min_node= list(node_l.keys())[0]
    else:
        #node_lリスト中の最小値のkeyを表示
        min_node = min(node_l, key=node_l.get)
    print('min_node: ', min_node)
    
    del_list = []
    for key1 in edge:
        for key2 in edge[key1]:
            """
            終点ノードがmin_nodeであるエッジを削除リストに追加
            """
            if key2 == min_node:
                del_list.append(key1+key2)
    
    print('del_list:', del_list)
    """
    削除リストにあるエッジをedge[]から削除
    """
    for keys in del_list:
        del edge[keys[0]][keys[1]]
    
    
    """
    min_nodeからつながっているノードの全てについて
    """
    for id_node, weight in edge[min_node].items():
        """
        ノードの現在のコストよりも、min_nodeのコストとそこからのエッジコストの和が小さければ更新
        """
        if (weight + node_l[min_node]) < node_l[id_node]:
            node_l[id_node] = weight + node_l[min_node]
            prev_node[id_node] = min_node
        
    """
    min_nodeから伸びたエッジはすべて確認したのでそのエッジを削除し、min_nodeノードの値を確定する
    """
    node_l_[min_node] = node_l[min_node]
    if len(edge) == 1:
        break
    del edge[min_node]
    del node_l[min_node]
    
    print('未探索エッジ', edge)
    print('未確定のノード', node_l)
    print('確定したノード', node_l_)
    print('*'*100)

print('確定したノード', node_l_)
print('prev_node', prev_node)
