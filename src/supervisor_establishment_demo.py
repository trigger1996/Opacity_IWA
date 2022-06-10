#!/usr/bin/python
# -*- coding:utf-8 -*-
from opacity_iwa.t_aic_2 import t_aic
import control_2_trajectory

from random import randint

event_uo = ['bbb', 'gbgu', 'ggbu', 'bgbu']
event_o  = ['bgb', 'gbg',  'gbb',  'ggb']
event_c  = ['bbb', 'gbgu', 'ggbu', 'bgbu', 'bgb', 'gbg',  'gbb',  'ggb']
event_uc = []


def main():

    t_aic_case_3 = t_aic('/home/ubuntu484/catkin_ws/src/opacity_ts_2022/src/opacity_iwa/iwa_TS/team_3_robots.yaml', ['(1,3,4)'], event_c, event_o, event_uc, event_uo)

    t_aic_case_3.construct_T_AIC()

    t_aic_case_3.remove_all_revealing_states()

    # print the first supervisor with maximal length
    supervisor_case_3 = t_aic_case_3.find_all_supervisor()

    supervisor_len_list = [len(supervisor_t) for supervisor_t in supervisor_case_3]
    supervisor_to_use = None
    
    # case 1
    # choose the longest supervisor
    '''
    max_word_len = max(supervisor_len_list)
    for supervisor_t in supervisor_case_3:
        if len(supervisor_t) == max_word_len:
                supervisor_to_use = supervisor_t
                break
    '''
    
    # case 2
    # randomly pick a supervisor
    supervisor_to_use = supervisor_case_3[randint(1, supervisor_case_3.__len__())]

    print("Supervisor: ")
    for node_t in supervisor_to_use:
        print(node_t)

    traj_seq = control_2_trajectory.supervisor_2_region_seq(t_aic_case_3.iwa, supervisor_to_use)
    print('Trajectory sequence: ')
    print(traj_seq)

    map_iwa = control_2_trajectory.map()
    map_iwa.load_map_control_from_yaml('/home/ubuntu484/catkin_ws/src/opacity_ts_2022/src/opacity_iwa/iwa_TS/map.yaml')

    pos_xy_list = []
    for node_t in traj_seq:
        pos_xy_list.append(map_iwa.iwa_state_2_pos(node_t))

    print('Pos_XY sequence: ')
    print(pos_xy_list)

    t_aic_case_3.plot()

if __name__ == '__main__':
    main()
    print('Finished')
