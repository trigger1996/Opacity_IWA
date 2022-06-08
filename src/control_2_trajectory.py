# -*- coding:utf-8 -*-
import io
import yaml
import networkx as nx
from math import sqrt

def state_type(state):
    if type(state[0]) == tuple and list(state).__len__() == 2:
        return 'Z_state'
    else:
        return 'Y_state'

def is_reached_target(bot_x, bot_y, target_x, target_y, dist_threshold=0.35):
    dist = sqrt((bot_x - target_x) ** 2 + (bot_y - target_y) ** 2)
    if dist <= dist_threshold:
        return True
    else:
        return False


def supervisor_2_region_seq(iwa, supervisor_in):
    region_seq = []
    node_last = None
    for node_t in supervisor_in:
        if state_type(node_t) == 'Y_state':
            region_seq.append(node_t[0])
        else:
            state_num_in_z = list(node_t[0]).__len__()
            for state_t in list(node_t)[0]:
                if state_type(node_last) == 'Y_state' and state_t == node_last[0] and state_num_in_z == 1:      # if last node is a Y-state, current state is identical to the Y-state, and the current state is the only states
                    region_seq.append(state_t)
                    break
                elif state_type(node_last) == 'Z_state' and state_t == node_last[0][0] and \
                     state_num_in_z == 1 and list(node_last[0]).__len__() == 1:                                 # if last node is a Z_state, current state is a Z_state, and
                    region_seq.append(state_t)
                    break
                elif state_type(node_last) == 'Z_state' and state_t == node_last[0][0] and state_num_in_z > 1:
                    continue
                else:
                    try:
                        if state_t in iwa.edge[region_seq[region_seq.__len__() - 1]]:                           # if the current state is one-step reachable
                            region_seq.append(state_t)
                            break
                    except:
                        continue

        node_last = node_t

    return region_seq

class map:
    def __init__(self):
        self.robot_num = 0
        self.node_pos = {}
        self.node_offset = {}
        self.robot_list = []
        pass

    def load_map_control_from_yaml(self, fin):
        fin = open(fin, 'r')
        data = yaml.load(fin)

        self.robot_num = data['robot_num']

        for pos_t in data['position']:
            self.node_pos[pos_t] = data['position'][pos_t]

        for robot_t in data['offset']:
            self.robot_list.append(robot_t)
            self.node_offset[robot_t] = data['offset'][robot_t]

        self.robot_list.sort()

    def iwa_state_2_pos(self, node):
        node_list = list(node)
        pos_list  = []
        for i in range(0, self.robot_list.__len__()):
            pos_index = node_list[1 + i * 2]
            x = self.node_pos[pos_index][0]
            y = self.node_pos[pos_index][1]

            x += self.node_offset[self.robot_list[i]][0]
            y += self.node_offset[self.robot_list[i]][1]

            pos_list.append([x, y])

        return pos_list


