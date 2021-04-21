#!/usr/bin/env python3
 
import rospy
import numpy as np
import tf
from pyproj import Proj
import math
from sensor_msgs.msg import Imu
from scout_msgs.msg import ScoutMotorState
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance


class Converter:
    def __init__(self, zone=52):
    
        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)
        
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)

        self.x, self.y, self.heading = None, None, None


    def imu_callback(self, imu_msg):
        
        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w

        (_, _, yaw) = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])

        self.heading = yaw

        print("/odom : ", yaw)


    def navsat_callback(self, gps_msg):
        
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude

        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.convertLL2UTM()

        if self.heading is not None:

            br = tf.TransformBroadcaster()

            br.sendTransform((self.x, self.y, 0.),
                            tf.transformations.quaternion_from_euler(0, 0, self.heading),
                            rospy.Time.now(),
                            "base_link",
                            "map")

    def convertLL2UTM(self):
    
        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0]-self.e_o
        self.y = xy_zone[1]-self.n_o


class CMDPublisher:
    def __init__(self):

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.u = np.zeros((2,))

        self.cmd_msg = Twist()

        self.t = 0.

    def calc_u(self):

        self.u[0] = 0.2

        self.u[1] = 0.5*np.sin(self.t*2*np.pi/5)

        self.cmd_msg.angular.z = self.u[1]

        self.cmd_msg.linear.x = self.u[0]

        self.t+=0.05

        self.cmd_pub.publish(self.cmd_msg)

        return self.u


class UnscentedKalmanFilter:
    def __init__(self, dt=0.05):
        
        self.T = dt

        self.nx = 3
        
        self.X = np.array([0,0,0], dtype=float).reshape([-1,1])
        
        self.P = np.diag([2,2,2])
        
        self.Q = self.T*np.diag([0.01, 0.01, 0.02])

        self.R = np.diag([0.1, 0.1])

        self.alpha = 0.002
        self.kappa = 0
        self.beta = 2
        
        self.H = np.array(
            [
                [1,0,0],
                [0,1,0]
            ],
            dtype=float)
        
        self.pose_pub = rospy.Publisher('/pose_ekf', Odometry, queue_size=1)

        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/map'

        self.set_params()


    def set_params(self):

        self.lamb = self.alpha ** 2 * (self.nx + self.kappa) - self.nx
        
        self.gamma = math.sqrt(self.nx + self.lamb)

        self.wm = np.zeros((self.nx * 2 + 1, ))
        self.wm[0] = self.lamb / (self.nx + self.lamb)
        self.wm[1:] = 1.0 / (2 * (self.nx + self.lamb))
        
        self.wc = np.zeros((self.nx * 2 + 1, ))
        self.wc[0] = self.lamb / (self.nx + self.lamb) + (1 - self.alpha ** 2 + self.beta)
        self.wc[1:] = 1.0 / (2 * (self.nx + self.lamb))

    
    def gen_sigma(self):

        self.Xsigma = np.tile(self.X, (1, 2 * self.nx + 1))

        d, V = np.linalg.eig(self.P)

        self.Xsigma[:, 1:(self.nx+1)] += self.gamma*np.sqrt(d)*V
        self.Xsigma[:, (self.nx+1):] -= self.gamma*np.sqrt(d)*V
    

    def calc_P(self):

        P = self.Q

        dx = self.Xsigma - self.X

        for i in range(self.nx * 2 + 1):

            P = P + self.wc[i] * dx[:, i].reshape([self.nx, 1]) @ dx[:, i].reshape([1, self.nx])

        return P


    def limit_yaw(self):

        over_bool = self.Xsigma[2,:] < -np.pi

        self.Xsigma[2, over_bool] = self.Xsigma[2, over_bool] + 2*np.pi

        under_bool = self.Xsigma[2,:] > np.pi

        self.Xsigma[2, under_bool] = self.Xsigma[2, under_bool] - 2*np.pi


    def prediction(self, u):

        self.gen_sigma()

        self.limit_yaw()

        dX_pre = np.zeros((self.nx, 2 * self.nx + 1), dtype=float)
        dX_pre[0, :] = u[0]*np.cos(self.Xsigma[2, :])
        dX_pre[1, :] = u[0]*np.sin(self.Xsigma[2, :])
        dX_pre[2, :] = u[1]
        
        self.Xsigma+=(self.T*dX_pre)

        self.limit_yaw()

        self.X = self.Xsigma @ self.wm.reshape([-1, 1])
        
        self.P = self.calc_P()


    def correction(self, Z):
    
        K = self.P.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
        Y = self.H.dot(self.X)
        
        self.X = self.X + K.dot(Z-Y)
    
        self.P = self.P - K.dot(self.H).dot(self.P)


    def send_estimated_state(self):

        pose_cov_ekf = PoseWithCovariance()

        q = tf.transformations.quaternion_from_euler(0, 0, self.X[2][0])

        pose_cov_ekf.pose.position.x = self.X[0][0]
        pose_cov_ekf.pose.position.y = self.X[1][0]
        pose_cov_ekf.pose.position.z = 0.

        pose_cov_ekf.pose.orientation.x = q[0]
        pose_cov_ekf.pose.orientation.y = q[1]
        pose_cov_ekf.pose.orientation.z = q[2]
        pose_cov_ekf.pose.orientation.w = q[3]

        P_3d = np.zeros((6,6))
        P_3d[:2, :2] = self.P[:2, :2]
        P_3d[-1, :2] = self.P[-1, :2]
        P_3d[:2, -1] = self.P[:2, -1]
        P_3d[-1, -1] = self.P[-1, -1]

        pose_cov_ekf.covariance = P_3d.reshape([-1]).tolist()

        self.odom_msg.pose = pose_cov_ekf

        self.pose_pub.publish(self.odom_msg)


if __name__ == '__main__':

    rospy.init_node('EKF_estimator', anonymous=True)
    
    rate = rospy.Rate(20)
    
    loc_sensor = Converter()
    
    ekf = UnscentedKalmanFilter(dt=0.05)
    
    cmd_gen = CMDPublisher()

    while not rospy.is_shutdown():
        
        if loc_sensor.x is not None and loc_sensor.y is not None:

            #decide the u
            u = cmd_gen.calc_u()
        
            #prediction step
            ekf.prediction(u)
        
            #measurement locations
            z = np.array([loc_sensor.x, loc_sensor.y]).reshape([-1,1])
        
            #correction step
            ekf.correction(z)
        
            #get the estimated states
            ekf.send_estimated_state()
        
        else:
            pass
        
        rate.sleep()