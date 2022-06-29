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
    self.caster = tf.TransformBroadcaster()
    

  def get_tf(self, ref_frame, obj):
    updated = False
    while updated==False:
      try:
        (trans,rot) = self.listener.lookupTransform(ref_frame, obj, rospy.Time(0))
        return trans, rot
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
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