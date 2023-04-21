#!/usr/bin/env python

import rospy
import rospkg
from distutils.version import LooseVersion
from franka_msgs.srv import SetEEFrame,SetEEFrameResponse
import numpy as np

# Workaround for https://github.com/frankaemika/franka_ros/issues/122
#
# Calling franka::Robot::setEE in libfranka 0.8.0 and above changes `NE_T_EE` instead of `F_T_EE`.
# Previously, this method would set the flange-to-end-effector transformation `F_T_EE`. 
# This has been split up into two transformations:
#  -  `F_T_NE`, only settable in Desk, and
#  - `NE_T_EE`, which can be set in `libfranka` defaults to the identity transformation.
# 
# This script detects the version of libfranka and sets either `F_T_EE` or `NE_T_EE` accordingly. 

# Partially based on https://github.com/justagist/franka_ros_interface/blob/master/franka_tools/src/franka_tools/frames_interface.py

class TcpProxy(object):

  Default_F_T_EE = [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426,
             0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.10339999943971634, 1.0]

  def __init__(self):

    self.set_EE_address = '/franka_control/set_EE_frame'
    self.proxy = rospy.Service('~set_EE_frame', SetEEFrame, self.proxy_callback)
    
  def get_libfranka_version(self):
      try:
        libfranka_version = rospkg.RosPack().get_manifest('libfranka').version
        rospy.logdebug("Set EE Frame Request: Detected libfranka: %s"%libfranka_version)
        return libfranka_version
      except rospkg.ResourceNotFound as e:
        rospy.logerr("Set EE Frame Request: Can not determine libfranka version: %s"%e)
        return False
  
  def proxy_callback(self, req):
     
      TCP_Full = self._assert_frame_validity(req.F_T_EE) 

      libfranka_version = self.get_libfranka_version()
      if not(libfranka_version):
        return SetEEFrameResponse(False,"Can not determine libfranka version")

      if LooseVersion(self.get_libfranka_version()) >= LooseVersion('0.8.0'):

        TCP_Gripper = np.reshape(a=self.Default_F_T_EE,newshape=[4,4],order='F')
        TCP_Full = np.reshape(a=TCP_Full,newshape=[4,4],order='F')
        
        F_T_EE = np.linalg.inv(TCP_Gripper)*TCP_Full
        rospy.loginfo("New libfranka. Sending NE_T_EE: \n%s"%F_T_EE)
      else:
        F_T_EE = TCP_Full
        rospy.loginfo("Legacy libfranka. Sending FE_T_EE: \n%s"%F_T_EE)

      if self.set_EE_frame(TCP_Full):
        return SetEEFrameResponse(True,"TCP set")
      else:
        return SetEEFrameResponse(True,"TCP not set")

  def set_EE_frame(self, frame):
      """
      Set new EE frame based on the transformation given by 'frame', which is the 
      transformation matrix defining the new desired EE frame with respect to the flange frame.
      :type frame: [float (16,)] / np.ndarray (4x4) 
      :param frame: transformation matrix of new EE frame wrt flange frame (column major)
      :rtype: bool
      :return: success status of service request
      """
      frame = self._assert_frame_validity(frame)
      return self._request_setEE_service(frame)

  def _request_setEE_service(self, trans_mat):

      rospy.wait_for_service(self.set_EE_address)
      try:
          service_handle = rospy.ServiceProxy(self.set_EE_address, SetEEFrame)
          response = service_handle(F_T_EE = trans_mat)
          rospy.loginfo("Set EE Frame Request Status: %s. \n\tDetails: %s"%("Success" if response.success else "Failed!", response.error))
          return response.success
      except rospy.ServiceException, e:
          rospy.logwarn("Set EE Frame Request: Service call failed: %s"%e)
          return False
      
  def _assert_frame_validity(self, frame):

        if isinstance(frame, np.ndarray):
            if frame.shape[0] == frame.shape[1] == 4:
                frame = frame.flatten('F').tolist()
            else:
                raise ValueError("Invalid shape for transformation matrix numpy array")
        else:
            assert len(frame) == 16, "Invalid number of elements in transformation matrix. Should have 16 elements."

        return frame
      
if __name__ == '__main__':
    try:
        rospy.init_node('tcp_proxy')
        TcpProxy()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
