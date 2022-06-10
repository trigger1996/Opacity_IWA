#!/usr/bin/python
# -*- coding:utf-8 -*-
import rospy
from bot_libs import bot_stl_ppc_physical


def main():
    rospy.init_node('opacity_ts_2022', anonymous=False)
    rate = rospy.Rate(60)	# 60Hz

    rospy.sleep(2)

    bot_1 = bot_stl_ppc_physical.bot_stl('rover_3')

    systick = 0

    while not rospy.is_shutdown():

        if systick % 20 == 0:
            print(bot_1.name, str([bot_1.x, bot_1.y, bot_1.yaw]), '    u:', str([bot_1.uv, bot_1.uw]))

        if bot_1.is_odom_updated:
            bot_1.update_other_vehicle_pos([[-0.35, -0.35, 0.5]])
            bot_1.move_end_to_end(0.5, 0.5, 15)
            bot_1.update_twist()

            bot_1.is_odom_updated = False

        systick = (systick + 1) % 2000
        rate.sleep()

if __name__ == '__main__':
    main()
    print('Finished')
