#!/usr/bin/env python

import rospy
import tf
import numpy as np
import argparse
import sys


class TfFilter():
    def __init__(self):

        parser = argparse.ArgumentParser(
            description=
            """Provide a filtered version of a desired transform. Ex. we have raw
            observations of Table1 frame in the camera frame of reference,
            this node will produce a transform from
            camera_frame to Table1_filtered.""")

        parser.add_argument(
            'child_frame',
            metavar='child_frame',
            type=str,
            help="""This is the raw child frame we are observing.
            Ex: Table1,
            raw observations of transform from camera to table frame""")

        parser.add_argument(
            'parent_frame',
            metavar='parent_frame',
            type=str,
            help="""This is the parent frame
            for the transform that we want to filter.""")

        parser.add_argument(
            '--destination_frame',
            metavar='destination_frame',
            type=str,
            help="""This is the name of the frame we will publish""",
            default=""
        )

        parser.add_argument(
            'min_observation_count',
            metavar='min_observation_count',
            type=str,
            help="""The number of required observations before
            we begin filtering the transform.""")

        parser.add_argument(
            '--name',
            type=str,
            default="pc_filter",
            help="""This is the ros node name""")

        # old args: /abc /xyz __name:=node_name __log:=my_logfile.log]
        # new args: /abc /xyz --name node_name --log my_logfile.log]
        args_list = " ".join(sys.argv[1:]).replace("__", "--").replace(
            ":=", " ").split(" ")

        args, _ = parser.parse_known_args(args_list)
        
        rospy.init_node(args.name)

        self.min_observation_count = args.min_observation_count

        # ex Table1
        self.observed_child_frame = args.child_frame
        # ex Table1_filtered

        if args.destination_frame != "":
            self.filtered_child_frame = args.destination_frame
        else:
            self.filtered_child_frame = args.child_frame + "_filtered"

        # kinect2_rgb_optical_frame (frame in which Table1 was observed from)
        self.parent_frame = args.parent_frame

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        # This is a moving average of the translational component
        # of the transform we are filtering
        self.filtered_trans = None
        # This is a moving average of the rotational component
        # of the transform we are filtering
        self.filtered_rot = None
        # This is the rate that our tranform will be published at
        self.rate = rospy.Rate(10.0)
        # number of times the raw transform has been observed
        self.observation_count = 0
       
       

    def _observe_raw_tf(self):
        raw_translation = None
        raw_rotation = None
        try:
            (raw_translation, raw_rotation) = self.tf_listener.lookupTransform(
                self.parent_frame, self.observed_child_frame,  rospy.Time(0))
            if self.observation_count < self.min_observation_count:
                self.observation_count += 1
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, tf.Exception), e:
            rospy.logerr("Failed to lookup transform for %s to %s" %
                         (self.parent_frame, self.observed_child_frame))

        return raw_translation, raw_rotation

    def _update_filtered_tf(self, raw_translation, raw_rotation):
        # If this is the first time we have received a valid
        # transformation, define filtered_rot + filtered_trans
        if self.filtered_rot is None or self.observation_count < 20:
            self.filtered_rot = raw_rotation
        if self.filtered_trans is None or self.observation_count < 20:
            self.filtered_trans = np.array(raw_translation)

        # Actual filtering of the transformation
        # between the observed and published transform
        self.filtered_rot = tf.transformations.quaternion_slerp(
            self.filtered_rot, raw_rotation, 0.01)
        self.filtered_trans = .99 * self.filtered_trans + 0.01 * np.array(
            raw_translation)

    def _broadcast_filtered_tf(self):
        if self.filtered_trans is None or self.filtered_rot is None:
            return

        self.tf_broadcaster.sendTransform(
            self.filtered_trans, self.filtered_rot,
            rospy.Time.now(), self.filtered_child_frame, self.parent_frame)

    def run(self):
        rospy.loginfo("Starting publish of transform")

        while not rospy.is_shutdown():
            # get a new observation of the raw transform if available
            # returns None, None if tf timeout
            raw_translation, raw_rotation = self._observe_raw_tf()

            # update our filtered version of the transform
            if raw_translation and raw_rotation:
                self._update_filtered_tf(raw_translation, raw_rotation)

            # broadcast the filtered transform
            self._broadcast_filtered_tf()

            self.rate.sleep()


if __name__ == '__main__':
    filter = TfFilter()
    filter.run()
