#!/usr/bin/python
# -*- coding:utf-8 -*-
import rospy
from bot_libs import bot_stl_ppc


def main():
    rospy.init_node('opacity_ts_2022', anonymous=False)
    rate = rospy.Rate(60)	# 60Hz

    rospy.sleep(2)

    bot_1 = bot_stl_ppc.bot_stl('rover_1')

    while not rospy.is_shutdown():

        if bot_1.is_odom_updated:
            bot_1.update_other_vehicle_pos([[2, 2, 0.5]])
            bot_1.move_end_to_end(5, 5, 15)
            bot_1.update_twist()

            bot_1.is_odom_updated = False

        rate.sleep()

if __name__ == '__main__':
    main()
    print('Finished')
