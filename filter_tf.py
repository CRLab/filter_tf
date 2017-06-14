#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
import tf_conversions


class TfFilter():

    def __init__(self):
        rospy.init_node('tf_filter')
        self.observed_frame = rospy.get_param("observed_frame") #/ar_marker_8 or _9
        self.filtered_frame = rospy.get_param("filtered_frame") #/world
        self.local_frame = rospy.get_param("local_frame") #camera_link or kinect2_link

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.filtered_trans = None
        self.filtered_rot = None
        self.rate = rospy.Rate(10.0)


    def run(self):
        rospy.loginfo("Starting publish of transform")
        
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform( self.observed_frame, self.local_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), e:
                rospy.logerr("Failed to lookup transform for %s to %s" % (self.local_frame, self.observed_frame))
                if self.filtered_rot is None:
                    #If we have never observed a valid transform, we cannot publish anything
                    self.rate.sleep()
                    continue
                else:
                    #We could not receive a new but publishing the previous value of this transform
                    self.tf_broadcaster.sendTransform(self.filtered_trans,
                                                      self.filtered_rot,
                                                      rospy.Time.now(),
                                                      self.local_frame,
                                                      self.filtered_frame)
                    self.rate.sleep()
                continue

            #If this is the first time we have received a valid transformation, define filtered_rot + filtered_trans
            if self.filtered_rot is None:
                self.filtered_rot = rot
            if self.filtered_trans is None:
                self.filtered_trans = np.array(trans)

            #Actual filtering of the transformation between the observed and published transform
            self.filtered_rot = tf.transformations.quaternion_slerp(self.filtered_rot, rot, 0.01)
            self.filtered_trans = .99 * self.filtered_trans + 0.01 * np.array(trans)

            self.tf_broadcaster.sendTransform(self.filtered_trans,
                                              self.filtered_rot,
                                              rospy.Time.now(),
                                              self.local_frame,
                                              self.filtered_frame)
            self.rate.sleep()

if __name__ == '__main__':
    filter = TfFilter()
    filter.run()
