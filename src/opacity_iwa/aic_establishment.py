#!/usr/bin/python
# -*- coding:utf-8 -*-
import networkx as nx
import io
import yaml
import copy
import matplotlib.pyplot as plt
from t_aic import t_aic, state_type

event_uo = ['a',  'b',  'uc']
event_o  = ['o1', 'o2', 'o3']
event_c  = ['a',  'b',  'o3']
event_uc = ['o1', 'o2', 'uc']

def assign_node_colors(bts):
    values = []
    for node_t in bts.nodes():
        if node_t == ('init', ):
            values.append('#DC5712')                # https://www.sioe.cn/yingyong/yanse-rgb-16/
        elif state_type(node_t) == 'Z_state':
            values.append('#FE4365')                # https://www.icoa.cn/a/512.html
        else:
            values.append('#83AF9B')
    return values

def main():

    #
    # CASE 3
    fin = open('/home/ghost/catkin_ws/src/opacity_ts_2022/src/opacity_iwa/iwa/IWA_3.yaml', 'r')
    data = yaml.load(fin)

    iwa = nx.MultiDiGraph()       # Graph MultiGraph
    for node_t in data['graph']['nodes']:
        iwa.add_node(node_t)
    for edge_t in data['graph']['edges']:
        event = edge_t[2]['event']
        t_min = edge_t[2]['t_min']
        t_max = edge_t[2]['t_max']
        iwa.add_edge(edge_t[0], edge_t[1], event=event, t_min=t_min, t_max=t_max)

    init_state = ['1']                                                  # ['0', '6'], ['6'] ['0'] ['8']
    bts = t_aic(iwa, init_state, event_uo, event_o, event_c, event_uc)  # iwa, ['0', '6'], event_uo, event_o, event_c, event_uc

    bts.add_edge(('init', ), tuple(init_state))
    node_color = assign_node_colors(bts)

    pos = nx.shell_layout(bts)   # nx.spectral_layout(bts) shell_layout spring_layout
    nx.draw(bts, pos=pos, with_labels=True, node_color=node_color, font_size=8.5)                        # https://www.jianshu.com/p/e254cd6acfdc/
                                                                                                         # https://blog.csdn.net/HsinglukLiu/article/details/107821649
                                                                                                         # https://www.cnpython.com/qa/39393

    nx.draw_networkx_edge_labels(bts, pos, font_size=6.5)                                                # https://blog.csdn.net/u013576018/article/details/60871485        # font_size=4.5

    plt.show()

if __name__ == '__main__':
    main()
    print('Finished')
