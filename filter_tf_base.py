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

        self.z_vec = None
        filtered_trans = None
        filtered_rot = None

        camera_frame = rospy.get_param("frame_id")

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (world_raw_trans, world_raw_rot) = listener.lookupTransform(camera_frame, '/world_raw', rospy.Time(0))
                (base_link_trans, base_link_rot) = listener.lookupTransform(camera_frame, '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            if filtered_rot is None:
                filtered_rot = world_raw_rot
            if filtered_trans is None:
                filtered_trans = np.array(world_raw_trans)

            filtered_rot = tf.transformations.quaternion_slerp(filtered_rot, world_raw_rot, 0.01)
            filtered_trans = .99 * filtered_trans + 0.01 * np.array(world_raw_trans)

            r = tf_conversions.Rotation.Quaternion(*base_link_rot)
            frame = tf_conversions.Frame(r, tf_conversions.Vector(0,0,0))
            mat = tf_conversions.toMatrix(frame)
            self.z_vec = mat[0:3, 2]

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

            world_raw_trans = filtered_trans
            world_raw_rot = np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]);

            br.sendTransform(world_raw_trans,
                             world_raw_rot,
                             rospy.Time.now(),
                             "/world",
                             camera_frame)
            rate.sleep()


if __name__ == '__main__':
    filter = TfFilter()
