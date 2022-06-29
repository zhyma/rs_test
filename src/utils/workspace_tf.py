# Covert raw RealSense `/camera/depth/image_rect_raw` data to Open3D point cloud data
# Run this first: `roslaunch realsense2_camera rs_camera.launch`

import sys
import rospy
import tf
# import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix, quaternion_matrix

## workspace tf
class workspace_tf:

  def __init__(self, rate):
    self.listener = tf.TransformListener()
    self.caster = tf.TransformBroadcaster()
    self.rate = rate
    

  def get_tf(self, ref_frame, obj):
    ## return a homogeneous transformation matrix
    updated = False
    while updated==False:
      try:
        (trans,rot) = self.listener.lookupTransform(ref_frame, obj, rospy.Time(0))
        h = quaternion_matrix(rot)
        h[:3,3] = trans
        return h
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        self.rate.sleep()


  def set_tf(self, ref_frame, obj, h, delay=1):
    updated = False
    q = quaternion_from_matrix(h)
    o = h[:3,3]
    # while updated==False:
    self.caster.sendTransform(o, q, rospy.Time.now(), obj, ref_frame)
      # rospy.sleep(delay)
      # try:
      #   _,_ = self.listener.lookupTransform(ref_frame, obj, rospy.Time(0))
      #   updated = True
      # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      #   rate.sleep()

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