#!/usr/bin/python

from face_id.face_id import CameraNode
import rospy

if __name__ == '__main__':
    rospy.init_node("Face")
    cn = CameraNode()
    cn.detect()
    
