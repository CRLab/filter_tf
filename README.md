# filter_tf
Simple Utility Node to filter a specified ROS TF transform


## Usage:
args are child_frame, parent_frame. From command line:
```
rosrun filter_tf filter_tf.py /ar_marker_8 /kinect2_link
```
In launch file:
```
<node pkg="filter_tf" type="filter_tf.py" name="filter_tf_marker8_kinec2" args="/ar_marker_8 /kinect2_link"/>
```

This will now broadcast a transform from /kinect2_link to /ar_marker_8_filtered.  This transform will continue to broadcast even if the raw transform becomes occluded.  This transform will be updated via a moving average whenever a new observation of the raw transform occurs.
