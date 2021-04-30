#!/usr/bin/env python3
 
import rospy
import numpy as np
import tf
from pyproj import Proj

from sensor_msgs.msg import Imu
from scout_msgs.msg import ScoutMotorState
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance, PoseArray
from std_msgs.msg import Float64MultiArray
from scout_msgs.msg import ScoutStatus
import matplotlib.pyplot as plt


def pdf_multivariate_gauss(dx, cov):

    part2 = np.exp(-0.5*np.matmul(np.matmul(dx.T, np.linalg.inv(cov)), dx))

    part1 = 1 / ((2* np.pi)**(cov.shape[0]/2) * (np.linalg.det(cov)**(1/2)))
    
    return np.diag(part1 * part2) + 0.000000001


class CMDParser:
    def __init__(self):
        
        self.status_sub = rospy.Subscriber("/scout_status", ScoutStatus, self.status_callback)

        self.u = np.zeros((2,))

    def status_callback(self, msg):
        
        self.u[0] = msg.linear_velocity
        
        self.u[1] = msg.angular_velocity


class LMparser:
    def __init__(self):

        self.lm_sub = rospy.Subscriber("/landmark_local", Float64MultiArray, self.lm_callback)

        self.lm_measure = None

    def lm_callback(self, msg):

        lm_list = msg.data

        self.lm_measure = lm_list



class ParticleFilter:
    def __init__(self, dt=0.05, NP=3000):
        
        self.T = dt

        self.NP = NP
        
        self.XP = np.random.randn(3, self.NP)*20

        self.limit_yaw()

        self.pw = np.ones((NP, ))/NP 

        self.q = 0.02

        self.R = np.diag([0.2, 0.2])/self.T

        # self.lm_list = [[0.0, -1.8], [0.0, 1.8], [5, 0.0]]
        self.lm_list = [[0.0, 1.8], [5, 0.0]]

        self.pose_pub = rospy.Publisher('/pose_pf', PoseArray, queue_size=1)

        self.update_bool = False

        self.Xm = None


    def prediction(self, u):

        dX_pre = np.zeros((3, self.NP), dtype=float)
        dX_pre[0, :] = u[0]*np.cos(self.XP[2, :])
        dX_pre[1, :] = u[0]*np.sin(self.XP[2, :])
        dX_pre[2, :] = u[1] + self.q*np.random.randn(1, self.NP)
        
        self.XP+=(self.T*dX_pre)

        self.limit_yaw()


    def limit_yaw(self):

        over_bool = self.XP[2,:] < -np.pi

        self.XP[2, over_bool] = self.XP[2, over_bool] + 2*np.pi

        under_bool = self.XP[2,:] > np.pi

        self.XP[2, under_bool] = self.XP[2, under_bool] - 2*np.pi


    def re_sampling(self, w):

        re_sample_ID = np.random.choice(np.arange(self.NP), self.NP, p=w)

        w = np.ones((self.NP, ))/self.NP 

        return re_sample_ID, w


    def landmark_measurement_model(self, lm_g):

        cth = np.cos(self.XP[2, :])
        sth = np.sin(self.XP[2, :])

        lm_local = np.zeros((2, self.NP))

        lm_local[0, :] = lm_g[0]*cth + lm_g[1]*sth - self.XP[0, :]*cth - self.XP[1, :]*sth
        lm_local[1, :] = -lm_g[0]*sth + lm_g[1]*cth + self.XP[0, :]*sth - self.XP[1, :]*cth

        return lm_local


    def correction(self, Z):

        if len(Z)!=0:

            Z = np.array(Z).reshape([-1, 3])

            print("landmark : ", Z)

            pw = self.pw

            lm_local = np.zeros((len(self.lm_list), 2, self.NP), dtype=np.float32)

            for id_lm in range(len(self.lm_list)):

                lm_local[id_lm, :, :] = self.landmark_measurement_model(self.lm_list[id_lm])

            for iz in range(Z.shape[0]):

                pdf_z = np.zeros((len(self.lm_list), self.NP), dtype=np.float32)

                for id_lm in range(len(self.lm_list)):

                    dz = lm_local[id_lm, :, :]-Z[iz, :2].reshape([2, -1])

                    pdf_z[id_lm, :] = pdf_multivariate_gauss(dz, self.R)

                pw = pw*np.max(pdf_z, axis=0)

            pw = pw / pw.sum()

            self.Xm = np.dot(self.XP, pw).reshape([-1, 1])

            Xcov = np.dot(
                (self.XP-self.Xm),
                np.diag(pw)
                ).dot((self.XP-self.Xm).T)

            re_sample_ID, self.pw = self.re_sampling(pw)

            XP_new = self.XP[:, re_sample_ID]

            self.XP = XP_new + self.T*np.diag([0.3, 0.3, 0.02]).dot(np.random.randn(3, self.NP))

            self.limit_yaw()

        else:
    
            self.Xm = np.dot(self.XP, self.pw).reshape([-1, 1])

            Xcov = np.dot(
                (self.XP-self.Xm),
                np.diag(self.pw)
                ).dot((self.XP-self.Xm).T)

        self.update_bool = True

    def send_estimated_state(self):
        
        particles_msg = PoseArray()
        particles_msg.header.frame_id='/map'

        for p_i in range(self.NP):

            particle_msg = Pose()

            px = self.XP[:, p_i]

            q = tf.transformations.quaternion_from_euler(0, 0, px[2])

            particle_msg.position.x = px[0]
            particle_msg.position.y = px[1]
            particle_msg.position.z = 0.

            particle_msg.orientation.x = q[0]
            particle_msg.orientation.y = q[1]
            particle_msg.orientation.z = q[2]
            particle_msg.orientation.w = q[3]

            particles_msg.poses.append(particle_msg)
            
        self.pose_pub.publish(particles_msg)


if __name__ == '__main__':

    rospy.init_node('PF_localization', anonymous=True)
    
    rate = rospy.Rate(10)
    
    pf = ParticleFilter(dt=0.1, NP=500)
    
    # cmd_gen = CMDPublisher()

    # u_parser = CMDParser()
    u_gen = CMDParser()

    lm_parser = LMparser()

    while not rospy.is_shutdown():
        
        if lm_parser.lm_measure is not None:

            #decide the u
            u = u_gen.u
        
            #prediction step
            pf.prediction(u)
        
            #measurement locations
            z = lm_parser.lm_measure
        
            #correction step
            pf.correction(z)
        
            #get the estimated states
            pf.send_estimated_state()
        
        else:
            pass
        
        rate.sleep()