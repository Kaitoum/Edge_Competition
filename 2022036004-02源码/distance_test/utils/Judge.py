#! /usr/bin/env python3

# python2
import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from perception_msgs.msg import Obstacle, ObstacleArray
# from ros_numpy import msgify
# from cv_bridge import CvBridge
import sys

sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
from socketserver import BaseRequestHandler, ThreadingTCPServer
import threading
import time
import numpy as np
import pzjc2 as cd

BUF_SIZE = 5000
# python3

global_data = "wo shi ni die" * 100


class SubscribeAndPublish:
    def __init__(self):
        self.all_obstacle_str = ''

        self.sub_marker_array_name = rospy.get_param("~sub_marker_array_name")
        print(self.sub_marker_array_name)
        self.sub_marker_array = rospy.Subscriber(self.sub_marker_array_name, MarkerArray, self.callback_marker_array)
        self.id_array = np.zeros(100, np.float32)
        self.id_arrayp = np.zeros(100, np.float32)
        self.infor = np.zeros((100, 10), np.float64)
        self.inforp = np.zeros((100, 10), np.float64)
        self.deltatime = 0.1

        #
        self.pub_marker_array_name = rospy.get_param("~pub_collision_array_name")
        self.pub = rospy.Publisher(self.pub_marker_array_name, MarkerArray, queue_size=20)

    def make_empty_marker_cylinder(self):
        marker = Marker()
        marker.header.frame_id = 'collision'
        marker.header.stamp = rospy.Time.now()
        marker.id = 0  # enumerate subsequent markers here
        marker.action = Marker.ADD  # can be ADD, REMOVE, or MODIFY
        marker.ns = "collision"
        marker.type = Marker.CYLINDER

        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1  # artifact of sketchup export
        marker.scale.y = 1  # artifact of sketchup export
        marker.scale.z = 1  # artifact of sketchup export

        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1.0

        # marker.lifetime = rospy.Duration()  # will last forever unless modifie
        marker.lifetime = rospy.Duration(0.1)  # will last forever unless modifie
        marker.text = "COLLISION"
        return marker

    def callback_marker_array(self, mka):

        # thistime=time.time()
        # print('time',thistime)
        print('')
        print('rec obs')
        num = len(mka.markers)
        Object = np.zeros((num, 10), np.float32)
        state_collision = np.zeros(num, np.int8)
        print('num of markers', num)
        if num > 0:
            for i in range(0, num):
                one_str = self.one_m_to_str(mka.markers[i])
                # get information of object
                id = mka.markers[i].id
                id = int(id)
                px = mka.markers[i].pose.position.x
                py = mka.markers[i].pose.position.y
                ox = mka.markers[i].scale.x
                oy = mka.markers[i].scale.y
                r = max(ox, oy) / 2
                state = 0
                statep = 0
                for k in range(0, len(self.id_arrayp)):  # judge whether object has appeared once
                    if self.id_arrayp[k] == id:
                        statep = k + 1
                for j in range(0, len(self.id_array)):  # judge whether object has appeared twice
                    if self.id_array[j] == id:
                        state = j + 1
                # print(statep,state)
                if statep == 0:
                    vx = 0
                    vy = 0
                    ax = 0
                    ay = 0
                    self.inforp[i] = [id, px, py, 0, 0, 0, 0, r, ox, oy]
                    self.infor[i] = [id, 0, 0, 0, 0, 0, 0, r, ox, oy]
                    self.id_arrayp[i] = id
                    self.id_array[i] = 0
                if state == 0 and statep:  # Differential calculation speed
                    vx = (px - self.inforp[statep - 1][1]) / self.deltatime
                    vy = (py - self.inforp[statep - 1][2]) / self.deltatime
                    ax = 0
                    ay = 0
                    self.infor[i] = [id, px, py, vx, vy, ax, ay, r, ox, oy]
                    self.id_arrayp[i] = id
                    self.id_array[i] = id
                if state and statep:  # Differential calculation acceleration
                    vx = (px - self.infor[state - 1][1]) / self.deltatime
                    vy = (py - self.infor[state - 1][2]) / self.deltatime
                    ax = (vx - self.infor[state - 1][3]) / self.deltatime
                    ay = (vy - self.infor[state - 1][4]) / self.deltatime
                    self.inforp[i] = self.infor[i]
                    self.infor[i] = [id, px, py, vx, vy, ax, ay, r, ox, oy]
                print(id, px, py, vx, vy, ax, ay, r, ox, oy)
                Object[i] = [id, px, py, vx, vy, ax, ay, r, ox, oy]

            d = cd.Collision_Detection()
            state_collision, area_collision = d.collision_predict(Object)
            count = 0  # numbei of collision object
            Object_collision = []
            for j in range(0, num):
                if state_collision[j]:
                    print("collision object id", int(Object[j][0]), ":")
                    print('x:', Object[j][1], 'y:', Object[j][2], 'scale_x:', Object[j][8], 'scale_y:', Object[j][9])
                    count += 1
            Object_collision = np.zeros((count, 5), np.float32)
            i = 0
            for j in range(0, num):
                if state_collision[j]:
                    Object_collision[i] = [Object[j][0], Object[j][1], Object[j][2], Object[j][8], Object[j][9]]
                    i += 1
            print(Object_collision)
            print("area_collision:", area_collision)
            # 发布collision信息
            test_collision_array = MarkerArray()

            for i in range(0, count):
                tmp = self.make_empty_marker_cylinder()
                tmp.id = int(Object_collision[i][0])
                tmp.pose.position.x = Object_collision[i][1]
                tmp.pose.position.y = Object_collision[i][2]
                tmp.scale.x = Object_collision[i][3]
                tmp.scale.y = Object_collision[i][4]
                test_collision_array.markers.append(tmp)
            # print("(len(area_collision)",(len(area_collision)))
            for i in range(len(area_collision)):
                tmp = self.make_empty_marker_cylinder()
                tmp.id = 0
                tmp.pose.position.x = area_collision[i][0]
                tmp.pose.position.y = area_collision[i][1]
                tmp.scale.x = area_collision[i][2]
                tmp.scale.y = area_collision[i][2]
                tmp.color.r = 0
                tmp.color.b = 1.0
                test_collision_array.markers.append(tmp)
            self.pub.publish(test_collision_array)
            print("publish collision once!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


def main():
    rospy.init_node('road_mks_tcp_server', anonymous=True)
    #####################
    # 开启ros
    t = SubscribeAndPublish()
    #####################
    rospy.spin()


if __name__ == "__main__":
    main()