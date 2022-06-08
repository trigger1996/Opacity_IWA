# encoding=utf-8

from numpy import gradient
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, fabs, sin, cos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
import numpy as np
import cvxopt
from math import exp, sqrt, atan2


class bot_stl:
    def __init__(self, name):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odom_cb)

        self.robot_traj_pub = rospy.Publisher('/' + self.name + '/path', Path, queue_size=1)
        self.tf_baselink_odom = TransformBroadcaster()
        self.robot_traj = Path()

        self.is_odom_updated = False

        self.obstacle_static = []
        self.obstacle_all = []
        self.other_vehicle = []

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.w = 0

        self.k1 = 0.55
        self.k2 = 1.45
        self.k3 = 1.85

        self.k1_ = self.k1
        self.k2_ = self.k2
        self.k3_ = 3.85

        self.turn_kp = 1
        self.turn_ki = 0
        self.turn_kd = 0

        self.yaw_err_int = 0
        self.yaw_err_int_max = 5

        self.uv = 0
        self.uw = 0

    def odom_cb(self, data):
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x,
                                                    data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z,
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.is_odom_updated = True

        # self.publish_tf_4_rviz()
        self.publish_path_4_rviz()

    def publish_tf_4_rviz(self):
        self.tf_baselink_odom.sendTransform((self.x, self.y, self.yaw),
                                            tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                                            rospy.Time.now(),
                                            self.name + "/base_link",
                                            "map")

    def publish_path_4_rviz(self):
        bot_pose_t = PoseStamped()

        bot_pose_t.header.stamp = rospy.Time.now()
        bot_pose_t.header.frame_id = '/' + self.name + '/odom'
        bot_pose_t.pose.position.x = self.x
        bot_pose_t.pose.position.y = self.y
        bot_pose_t.pose.position.z = 0
        [bot_pose_t.pose.orientation.x, bot_pose_t.pose.orientation.y, bot_pose_t.pose.orientation.z,
         bot_pose_t.pose.orientation.w] = quaternion_from_euler(0., 0., self.yaw)

        self.robot_traj.poses.append(bot_pose_t)
        self.robot_traj.header.stamp = rospy.Time.now()
        self.robot_traj.header.frame_id = '/' + self.name + '/odom'
        self.robot_traj_pub.publish(self.robot_traj)

    def turn(self, target_yaw):
        yaw_err = target_yaw - self.yaw
        self.yaw_err_int = self.yaw_err_int + yaw_err

        if self.yaw_err_int_max >= self.yaw_err_int_max:
            self.yaw_err_int_max = self.yaw_err_int_max
        elif self.yaw_err_int_max <= -self.yaw_err_int_max:
            yaw_err_int_max = -self.yaw_err_int_max

        self.uv = 0
        self.uw = self.turn_kp * yaw_err + self.turn_ki * self.yaw_int + self.turn.kd * self.w

    def move_end_to_end(self, x_goal, y_goal, t):
        # Target && STL Barrier
        b = 125 * exp(-0.2295 * t) + 5 - sqrt((self.x - x_goal) ** 2 + (self.y - y_goal) ** 2)
        yita = -125 * exp(-0.2295 * t) + 5
        h = 15 - sqrt((self.x - x_goal) ** 2 + (self.y - y_goal) ** 2)

        b_obs_arr = []
        for obstacle in self.obstacle_all:
            x_obs = obstacle[0]
            y_obs = obstacle[1]
            b_obs_t = sqrt((self.x - x_obs) ** 2 + (self.y - y_obs) ** 2) - 16          # sqrt((self.x - x_obs) ** 2 + (self.y - y_obs) ** 2) - 18
            b_obs_arr.append(b_obs_t)
        if not b_obs_arr.__len__():
            b_obs = 0
        else:
            b_obs = min(b_obs_arr)

        # Theta Generator
        yaw_r = atan2(y_goal - self.y, x_goal - self.x)
        b_yaw = abs(self.yaw - yaw_r)

        # STL Controller

        partial_b = (1. / 2) * (1. / sqrt((self.x - x_goal) ** 2 + (self.y - y_goal) ** 2))
        # partial_b_x = np.mat([-2*partial_b*(self.x - x_goal), -2*partial_b*(self.y - y_goal),0])
        partial_b_x = cvxopt.matrix([[-2 * partial_b * (self.x - x_goal)], [-2 * partial_b * (self.y - y_goal)], [0]])

        # gx = np.mat([[cos(self.yaw), 0], [sin(self.yaw), 0], [0, 1]])
        gx = cvxopt.matrix([[cos(self.yaw), sin(self.yaw), 0], [0, 0, 1]])

        A = partial_b_x * gx

        # H = np.mat([[1, 0], [0, 1]])
        H = cvxopt.matrix([[1.0, 0.0], [0.0, 1.0]])
        f = cvxopt.matrix([0.0, 0.0])                                                   # cvxopt.matrix([0.0, 1.0])

        b_stl = 15 * b - 0.2295 * 125 * exp(-0.2295 * t)
        # U_stl = quadprog(H,[0;1],-A,b_stl)

        options = {}
        options['show_progress'] = False
        result = cvxopt.solvers.qp(P=H, q=f, G=-A, h=cvxopt.matrix([b_stl]), options=options)
        U_stl = result['x']

        # STL Unicycle Controller
        xe = cos(self.yaw) * (x_goal - self.x) + sin(self.yaw) * (y_goal - self.y)      # xe
        ye = -sin(self.yaw) * (x_goal - self.x) + cos(self.yaw) * (y_goal - self.y)     # ye
        we = yaw_r - self.yaw;  # we
        vr = 0
        wr = 0

        Uv = vr * 1 + self.k1 * xe * sqrt(1 + xe ** 2 + ye ** 2) ** (-1)
        Uw = wr + self.k2 * vr * sqrt(1 + xe ** 2 + ye ** 2) ** (-1) * (
                    ye * cos(0.5 * we) - xe * sin(0.5 * we)) + self.k3 * sin(0.5 * we)

        #print(yaw_r, self.yaw)

        # STL regulation Function
        k = 100
        lambda_stl = 1 - exp(-k * max(b_yaw, 0))
        uv_stl = lambda_stl * Uv + (1 - lambda_stl) * U_stl[0]
        uw_stl = lambda_stl * Uw + (1 - lambda_stl) * U_stl[1]

        # Artificial Potential Field
        Ap = np.mat([[-1, 0], [0, -1]])
        F = np.mat([[2 * x_goal, 2 * y_goal]])
        [gradient_x, gradient_y] = self.calculate_gradient(self.x, self.y, Ap, F)

        yawr_p = atan2(gradient_y, gradient_x)
        xr_p = self.x + 30 * gradient_x
        yr_p = self.y + 30 * gradient_y

        # APF Unicycle Controller
        xe_p = cos(self.yaw) * (xr_p - self.x) + sin(self.yaw) * (yr_p - self.y)  # xe
        ye_p = -sin(self.yaw) * (xr_p - self.x) + cos(self.yaw) * (yr_p - self.y)  # ye
        we_p = yawr_p - self.yaw  # we
        vr_p = 0
        wr_p = 0

        Uv_p = vr_p * 1 + self.k1_ * xe_p * sqrt(1 + xe_p ** 2 + ye_p ** 2) ** (-1)
        Uw_p = wr_p + self.k2_ * vr_p * sqrt(1 + xe_p ** 2 + ye_p ** 2) ** (-1) * (
                    ye_p * cos(0.5 * we_p) - xe_p * sin(0.5 * we_p)) + self.k3_ * sin(0.5 * we_p)

        # STL & APF  Regulation Function
        k = 0.1                                                                            # 0.05
        lambda_all = 1 - exp(-k * max(b_obs, 0))

        self.uv = lambda_all * uv_stl + (1 - lambda_all) * Uv_p
        self.uw = lambda_all * uw_stl + (1 - lambda_all) * Uw_p

    def update_twist(self, v=None, w=None):
        if v == None or w == None:
            pass
        else:
            self.vx = v
            self.wz = w

        move_cmd = Twist()
        move_cmd.linear.x = self.uv
        move_cmd.angular.z = self.uw
        self.twist_pub.publish(move_cmd)

    def set_obstacle_pos(self, obstacle_pos):
        # [x, y, radius]
        del self.obstacle_static[:]
        for obstacle in obstacle_pos:
            self.obstacle_static.append([obstacle[0], obstacle[1], obstacle[2]])

    def update_other_vehicle_pos(self, vehicle_pos):
        # [x, y, radius]
        del self.other_vehicle[:]
        self.other_vehicle = []
        for vehicle in vehicle_pos:
            self.other_vehicle.append([vehicle[0], vehicle[1], vehicle[2]])

        del self.obstacle_all[:]
        self.obstacle_all = []
        for obstacle in self.obstacle_static:
            self.obstacle_all.append([obstacle[0], obstacle[1], obstacle[2]])
        for vehicle in self.other_vehicle:
            self.obstacle_all.append([vehicle[0], vehicle[1], vehicle[2]])

    def calculate_gradient(self, x, y, A, F):
        gradient_U1 = 0.001 * (2 * A * np.mat([[x], [y]]) + F.transpose())
        gradient_U2 = 0

        for obstacle in self.obstacle_all:
            x_o = obstacle[0]
            y_o = obstacle[1]
            r_o = obstacle[2]

            r = sqrt((x - x_o) ** 2 + (y - y_o) ** 2)
            D_q = (1. / 2) * ((x - x_o) ** 2 + (y - y_o) ** 2) ** (-1. / 2) * np.mat([[2 * (x - x_o)], [2 * (y - y_o)]])
            if r <= r_o:
                gradient_U2 = gradient_U2 - 2 * 3000 * ((1. / r_o) - (1. / r)) * (1. / (r ** 2)) * D_q

        U_all = gradient_U1 + gradient_U2
        gradient_x = float(U_all[0])
        gradient_y = float(U_all[1])

        return [gradient_x, gradient_y]
