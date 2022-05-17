# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import rospy
import tf
# import geometry_msgs.msg

## workspace tf
class workspace_tf:

  def __init__(self):
    self.listener = tf.TransformListener()
    self.tf_updated = False
    self.trans = []
    self.rot = []
    

  def get_tf(self):
    try:
      (self.trans,self.rot) = self.listener.lookupTransform('/camera_depth_frame','ar_marker_90', rospy.Time(0))
      self.tf_updated = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      self.tf_updated = False
      ...

if __name__ == '__main__':
  rospy.init_node('tf_converter', anonymous=True)
  ws_tf = workspace_tf()
  rate = rospy.Rate(3)
  while not rospy.is_shutdown():
    ws_tf.get_tf()
    if ws_tf.tf_updated:
      print(ws_tf.trans)
      print(ws_tf.rot)
      print("====")
      rate.sleep()
    else:
      print("marker not found")
      print("====")