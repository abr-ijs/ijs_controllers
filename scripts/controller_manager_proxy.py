#!/usr/bin/env python3

import rospy
import controller_manager_msgs.srv as cm_srv
import robot_module_msgs.srv as ijs_srv
import std_srvs.srv as std_srv

# Workaround for outdated message definitions in Matlab
# https://www.mathworks.com/matlabcentral/answers/539627-updating-ros-controller_manager_msgs-to-melodic

# Based on https://github.com/RoboticVisionOrg/rv_manipulation_driver/blob/master/src/rv_manipulation_driver/_control_switcher.py

class ControlSwitcher(object):
  def __init__(self, controller_manager_node='controller_manager', robot_resource_name='panda'):
    self.robot_resource_name = robot_resource_name
    self.controller_manager_node = controller_manager_node

    self.reconnect()

    self.__current = ''
    self.__last_update = rospy.get_time()
    
    self.active_controller = rospy.Service('~get_active_controller', std_srv.Trigger, self.active_cb)
    self.stop_controller = rospy.Service('~stop_controller', ijs_srv.SetString, self.stop_cb)
    self.start_controller = rospy.Service('~start_controller', ijs_srv.SetString, self.start_cb)
    
  def reconnect(self):
    rospy.wait_for_service(self.controller_manager_node + '/switch_controller')
    rospy.wait_for_service(self.controller_manager_node + '/list_controllers')
    
    self.switcher_srv = rospy.ServiceProxy(self.controller_manager_node + '/switch_controller', cm_srv.SwitchController)
    self.lister_srv = rospy.ServiceProxy(self.controller_manager_node + '/list_controllers', cm_srv.ListControllers)
    
  def get_current_name(self):
    try:
      current_time = rospy.get_time()

      if not self.__current or current_time - self.__last_update > 5:
        controllers = self.lister_srv().controller
        for controller in controllers:
            if controller.state != 'running':
              continue
            
            resources = [item for claimed in controller.claimed_resources for item in claimed.resources]
            
            if len(list(resource for resource in resources if resource.startswith(self.robot_resource_name+"_joint"))):
              self.__current = controller.name
              break

        self.__last_update = current_time
                  
      return self.__current

    except:
      rospy.logerr('Disonnected from controller_manager/list_controllers, reconnecting...')
      self.reconnect()
      return self.__current
      
#  def switch_controller(self, controller_name):
#      controllers = self.lister_srv().controller
#      selected = None
#      
#      for controller in controllers:
#        if controller.name == controller_name:
#          selected = controller
#          break
#
#      required = [item for claimed in controller.claimed_resources for item in claimed.resources]
#
#      start_controllers = [controller_name] if controller_name else []
#      stop_controllers = [self.get_current_name()] if not controller_name else []
#
#      for controller in controllers:
#          if controller.name == controller_name:
#            continue
#
#          resources = [item for claimed in controller.claimed_resources for item in claimed.resources]
#          
#          if len(list(resource for resource in resources if resource.startswith(self.robot_resource_name))):
#            stop_controllers.append(controller.name)
#      
#
#      controller_switch_msg = cm_srv.SwitchControllerRequest()
#      controller_switch_msg.strictness = 1
#      controller_switch_msg.start_controllers = start_controllers
#      controller_switch_msg.stop_controllers = stop_controllers
#
#      res = self.switcher_srv(controller_switch_msg).ok
#      if res:
#          rospy.loginfo('Successfully switched to controller %s' % (controller_name))
#          self.__current = controller_name
#          return res
#      else:
#          return False
            
  def stop_cb(self, req):
      controller_name = req.string
      controller_switch_msg = cm_srv.SwitchControllerRequest()
      controller_switch_msg.strictness = 1
      controller_switch_msg.stop_controllers = [ controller_name ]
      
      res = self.switcher_srv(controller_switch_msg).ok
      if res:
          rospy.loginfo('Successfully stopped controller %s' % (controller_name))
          self.__current = controller_name
          return ijs_srv.SetStringResponse(True,self.__current)
      else:
          return ijs_srv.SetStringResponse(False,self.__current)

  def start_cb(self, req):
      controller_name = req.string
      controller_switch_msg = cm_srv.SwitchControllerRequest()
      controller_switch_msg.strictness = 1
      controller_switch_msg.start_controllers = [ controller_name ]
      
      res = self.switcher_srv(controller_switch_msg).ok
      if res:
          rospy.loginfo('Successfully started controller %s' % (controller_name))
          self.__current = controller_name
          return ijs_srv.SetStringResponse(True,self.__current)
      else:
          return ijs_srv.SetStringResponse(False,self.__current)
      
  def active_cb(self, req):
      active_controller = self.get_current_name()
      return std_srv.TriggerResponse(True,active_controller)
      
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Controller helper')
    parser.add_argument('-r','--robot',default='panda')
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        rospy.init_node('controller_proxy')
        ControlSwitcher(robot_resource_name=args.robot)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
