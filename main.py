#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
import tf_conversions


class TfFilter():

    def __init__(self):
        rospy.init_node('tf_world_filter')

        listener = tf.TransformListener()
        br = tf.TransformBroadcaster()



        rospy.Subscriber("/table_detector/tablePose", geometry_msgs.msg.Vector3, self.callback)

        self.z_vec_raw = None
        self.z_vec = None
        filtered_trans = None
        filtered_rot = None
        
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/camera_rgb_optical_frame', '/world_raw', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
                
            if filtered_rot is None:
                filtered_rot = rot
            if filtered_trans is None:
                filtered_trans = np.array(trans)

            filtered_rot = tf.transformations.quaternion_slerp(filtered_rot, rot, 0.01)
            filtered_trans = .99 * filtered_trans + 0.01 * np.array(trans)

            if self.z_vec_raw is None:
                r = tf_conversions.Rotation.Quaternion(*filtered_rot)
                frame = tf_conversions.Frame(r, tf_conversions.Vector(0,0,0))
                mat = tf_conversions.toMatrix(frame)
                self.z_vec_raw = mat[0:3, 2]
                self.z_vec = self.z_vec_raw


            r = tf_conversions.Rotation.Quaternion(*filtered_rot)
            frame = tf_conversions.Frame(r, tf_conversions.Vector(0,0,0))
            mat = tf_conversions.toMatrix(frame)
            x = mat[0:3, 0]
            n = self.z_vec
            unit_n = n / np.linalg.norm(n)
            z_new = unit_n
            x_new = x - np.dot(x, unit_n)*unit_n
            x_new = x_new / np.linalg.norm(x_new)
            y_new = np.cross(z_new, x_new)

            rot_new = np.zeros((4, 4))
            rot_new[3, 3] = 1.0

            rot_new[0:3, 0] = x_new
            rot_new[0:3, 1] = y_new
            rot_new[0:3, 2] = z_new

            rot_new[0:3, 3] = filtered_trans

            frame = tf_conversions.fromMatrix(rot_new)
            pose_msg = tf_conversions.toMsg(frame)

            trans = filtered_trans
            rot = np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]);
            
            br.sendTransform(trans,
                             rot,
                             rospy.Time.now(),
                             "/world",
                             "/camera_rgb_optical_frame")
            rate.sleep()




    def callback(self, msg):
        self.z_vec_raw = (msg.x, msg.y, msg.z)
        self.z_vec = .9 * self.z_vec + 0.1 * np.array(self.z_vec_raw)

if __name__ == '__main__':
    filter = TfFilter()