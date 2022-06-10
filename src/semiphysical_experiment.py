#!/usr/bin/python
# -*- coding:utf-8 -*-
import rospy
from bot_libs import bot_stl_ppc

from opacity_iwa.t_aic_2 import t_aic
import control_2_trajectory

from random import randint

event_uo = ['bbb', 'gbgu', 'ggbu', 'bgbu']
event_o  = ['bgb', 'gbg',  'gbb',  'ggb']
event_c  = ['bbb', 'gbgu', 'ggbu', 'bgbu', 'bgb', 'gbg',  'gbb',  'ggb']
event_uc = []

vehicle_r = 0.55



def main():
    rospy.init_node('opacity_ts_2022', anonymous=False)
    rate = rospy.Rate(60)	# 60Hz

    rospy.sleep(2)

    bot_1 = bot_stl_ppc.bot_stl('rover_1')
    bot_2 = bot_stl_ppc.bot_stl('rover_2')
    bot_3 = bot_stl_ppc.bot_stl('rover_3')


    # BTS Establishment
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

    index = 0
    time  = 60

    systick = 0

    while not rospy.is_shutdown():
        
        # print current position
        if systick % 500 == 0:
            print(bot_1.name, str([bot_1.x, bot_1.y]), str(pos_xy_list[index][0]), control_2_trajectory.is_reached_target(bot_1.x, bot_1.y, pos_xy_list[index][0][0], pos_xy_list[index][0][1]))
            print(bot_2.name, str([bot_2.x, bot_2.y]), str(pos_xy_list[index][1]), control_2_trajectory.is_reached_target(bot_2.x, bot_2.y, pos_xy_list[index][1][0], pos_xy_list[index][1][1]))
            print(bot_3.name, str([bot_3.x, bot_3.y]), str(pos_xy_list[index][2]), control_2_trajectory.is_reached_target(bot_3.x, bot_3.y, pos_xy_list[index][2][0], pos_xy_list[index][2][1]))

        # if all robots arrived the destination, then update the waypoints
        if control_2_trajectory.is_reached_target(bot_1.x, bot_1.y, pos_xy_list[index][0][0], pos_xy_list[index][0][1]) and \
           control_2_trajectory.is_reached_target(bot_2.x, bot_2.y, pos_xy_list[index][1][0], pos_xy_list[index][1][1]) and \
           control_2_trajectory.is_reached_target(bot_3.x, bot_3.y, pos_xy_list[index][2][0], pos_xy_list[index][2][1]):
            index += 1
            time  += 60
           
            if index >= pos_xy_list.__len__():
                index = pos_xy_list.__len__() - 1
                time  = pos_xy_list.__len__() * 60
               
                if systick % 500 == 0:
                    print('Supervisor finished!')

                    bot_1.uv = 0
                    bot_1.uw = 0
                    bot_2.uv = 0
                    bot_2.uw = 0
                    bot_3.uv = 0
                    bot_3.uw = 0
                    continue
            else:
                if systick % 500 == 0:
                    print('All rover arrived destinated positions, ready for next!')

        # update control
        if bot_1.is_odom_updated:
            bot_1.update_other_vehicle_pos([[bot_2.x, bot_2.y, vehicle_r], [bot_3.x, bot_3.y, vehicle_r]])

            bot_1.move_end_to_end(pos_xy_list[index][0][0], pos_xy_list[index][0][1], time)
            bot_1.update_twist()

            bot_1.is_odom_updated = False

        if bot_2.is_odom_updated:
            bot_2.update_other_vehicle_pos([[bot_1.x, bot_1.y, vehicle_r], [bot_3.x, bot_3.y, vehicle_r]])

            bot_2.move_end_to_end(pos_xy_list[index][1][0], pos_xy_list[index][1][1], time)
            bot_2.update_twist()

            bot_2.is_odom_updated = False

        if bot_3.is_odom_updated:
            bot_3.update_other_vehicle_pos([[bot_1.x, bot_1.y, vehicle_r], [bot_2.x, bot_2.y, vehicle_r]])

            bot_3.move_end_to_end(pos_xy_list[index][2][0], pos_xy_list[index][2][1], time)
            bot_3.update_twist()

            bot_3.is_odom_updated = False

        systick = (systick + 1) % 2000
        rate.sleep()

if __name__ == '__main__':
    main()
    print('Finished')
