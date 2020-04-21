#!/usr/bin/env python

import rospy
import os
import numpy as np
import xml.etree.ElementTree as ET
import tf

# show object locations in rviz:
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Pose, TransformStamped
from nav_msgs.msg import Odometry


class ArtifactLocator:
   def __init__(self):
      self.potential_artifacts = ['backpack', 'phone', 'rescue_randy', 
                        'gas', 'vent', 'extinguisher' ]
      self.artifact_dictionary = dict()
      self.rate = rospy.Rate(5) # 5 Hz  
      self.curr_robot_pos = np.array([0.0, 0.0, 0.0])
      self.artifact_counter = 0
      self.found_artifacts = MarkerArray()

      self.dist_threshold = rospy.get_param('~dist_thresh', 5)
      self.agentName = rospy.get_param('~agent_name', 'X1')
      self.worldName = rospy.get_param('~world_name',"simple_tunnel_01")
      rospy.loginfo(self.agentName + " " + self.worldName + " " + str(self.dist_threshold))

      self.close_artifact_pub = rospy.Publisher('found_artifacts', MarkerArray, queue_size=1)
      self.robot_pose_sub = rospy.Subscriber(self.agentName + '/odom', Odometry, self.robot_pose_callback)
      self.populate_artifact_dictionary()   

   def populate_artifact_dictionary(self):
      result = self.find_files(self.worldName + ".sdf",os.path.expanduser("~/subt_ws"))
      root = ET.parse(result[0]).getroot()

      # Go through world file and get all element names and poses
      for name_element, pose_element in zip(root.iter('name'), root.iter('pose')):
         #Add any element which contains a substring from potential artifacts to artifact dict
         if any(x in name_element.text for x in self.potential_artifacts):
            pose = np.fromstring(pose_element.text, dtype=float, sep=' ')
            self.artifact_dictionary[name_element.text] = pose

   def find_files(self, filename, search_path):
      result = []
      # Walking top-down from the root
      for root, dir, files in os.walk(search_path):
         if filename in files:
            result.append(os.path.join(root, filename))
      return result
   
   def check_artifact_proximity(self):
      close_artifacts = []
      for name, pose in self.artifact_dictionary.items():
         dist = np.linalg.norm(self.curr_robot_pos - pose[0:3])
         if(dist < self.dist_threshold):
            close_artifacts.append(name)
            rospy.loginfo("name = %s dist = %f", name, dist)
      return close_artifacts

   def robot_pose_callback(self, msg):
      self.curr_robot_pos[0] = msg.pose.pose.position.x
      self.curr_robot_pos[1] = msg.pose.pose.position.y
      self.curr_robot_pos[2] = msg.pose.pose.position.z
      return 

   def publish_artifact_markers(self, close_artifacts):
      for artifact_name in close_artifacts:
         pose_array = self.artifact_dictionary[artifact_name]
         m = Marker()
         m.ns = artifact_name #Using the namespace to publish the artifacts name (Hacky...)
         m.header.frame_id = self.agentName + "/map"

         m.id = self.artifact_counter
         self.artifact_counter += 1
         m.action = m.ADD
         m.scale.x = 1.0
         m.scale.y = 1.0
         m.scale.z = 1.0        
         
         m.pose.orientation.w = 1.0
         m.pose.position.x = pose_array[0]
         m.pose.position.y = pose_array[1]
         m.pose.position.z = pose_array[2] + 0.5
         m.type = m.SPHERE
         m.color.r = 1; m.color.g = 0; m.color.b = 0; m.color.a = 1.0
         self.found_artifacts.markers.append(m)
      self.close_artifact_pub.publish(self.found_artifacts)

   def main_loop(self):
      while not rospy.is_shutdown():
         close_artifacts = self.check_artifact_proximity()
         self.publish_artifact_markers(close_artifacts)
         self.rate.sleep()  

if __name__ == '__main__':
   # Initialize the node
   rospy.init_node('artifact_locator')
   al = ArtifactLocator()
   al.main_loop()
      


