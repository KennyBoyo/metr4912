#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from franka_msgs.msg import FrankaState
import tf.transformations as tr



class equilibrium_publisher:

  def __init__(self):
    self.sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.equilibrium_adjuster_callback)
    self.pub = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=10)
    self.position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]
    self.robot_pose_eq = PoseStamped()
    self.robot_pose = PoseStamped()
    self.index = 0
    self.buffer_size = 10
    self.robot_pose_list = [None] * self.buffer_size
    self.robot_pose.header.frame_id = "panda_link0"


  def equilibrium_adjuster_callback(self, actual: FrankaState):
    self.robot_pose_list[self.index] = actual
    self.index += 1
    if (self.index == self.buffer_size):
      self.index = 0
    if self.robot_pose_list[self.index] != None:
      actual = self.robot_pose_list[self.index]
      self.robot_pose.header.stamp = rospy.Time.now()
      quat = tr.quaternion_from_matrix([[actual.O_T_EE[0], actual.O_T_EE[4], actual.O_T_EE[8], actual.O_T_EE[12]], \
        [actual.O_T_EE[1], actual.O_T_EE[5], actual.O_T_EE[9], actual.O_T_EE[13]], \
          [actual.O_T_EE[2], actual.O_T_EE[6], actual.O_T_EE[10], actual.O_T_EE[14]], \
            [actual.O_T_EE[3], actual.O_T_EE[7], actual.O_T_EE[11], actual.O_T_EE[15]]])
      self.robot_pose.pose.position.x = max([min([actual.O_T_EE[12],
                                            self.position_limits[0][1]]),
                                            self.position_limits[0][0]])
      self.robot_pose.pose.position.y = max([min([actual.O_T_EE[13],
                                        self.position_limits[1][1]]),
                                        self.position_limits[1][0]])
      self.robot_pose.pose.position.z = max([min([actual.O_T_EE[14],
                                        self.position_limits[2][1]]),
                                        self.position_limits[2][0]])
      self.robot_pose.pose.orientation.x = quat[0]
      self.robot_pose.pose.orientation.y = quat[1]
      self.robot_pose.pose.orientation.z = quat[2]
      self.robot_pose.pose.orientation.w = quat[3]
      self.pub.publish(self.robot_pose)



def main(args):
  rospy.init_node('equilibrium_publisher', anonymous=True)
  obc = equilibrium_publisher()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)