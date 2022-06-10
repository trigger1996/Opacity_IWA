#!/usr/bin/python
# -*- coding:utf-8 -*-
from opacity_iwa.t_aic_2 import t_aic

event_uo = ['a',  'b',  'uc']
event_o  = ['o1', 'o2', 'o3']
event_c  = ['a',  'b',  'o3']
event_uc = ['o1', 'o2', 'uc']


def main():

    bts = t_aic('/home/ghost/catkin_ws/src/opacity_ts_2022/src/opacity_iwa/iwa/IWA_3.yaml', ['1'], event_c, event_o, event_uc, event_uo)

    bts.construct_T_AIC()

    bts.remove_all_revealing_states()

    # print the first supervisor with maximal length
    supervisor_case_3 = bts.find_all_y_y_supervisor()

    supervisor_len_list = [len(supervisor_t) for supervisor_t in supervisor_case_3]
    max_word_len = max(supervisor_len_list)
    for supervisor_t in supervisor_case_3:
        if len(supervisor_t) == max_word_len:
                print(supervisor_t)
                break

    bts.plot()

if __name__ == '__main__':
    main()
    print('Finished')
