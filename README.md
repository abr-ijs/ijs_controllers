A collection of controllers for the research version of Franka Emika Robot (Panda) and Franka Emika Research 3 (FR3) robots using franka_ros and libfranka. 

Included controllers:
- task space impedance controller with full stiffness and damping  matrix
- joint space impedance controller with stiffness and damping parameters 
- joint velocity controller



## Launch on a real robot

- Launch on Panda:   `roslaunch ijs_controllers ijs_controller_helper.launch robot_ip:=<FCI_IP> arm_id:=panda_1 legacy_franka_ros:=true __ns:=panda_1 load_gripper:=true controller:=cartesian_impedance_controller`
- Launch on FR3:  `roslaunch ijs_controllers cartesian_impedance.launch robot_ip:=<FCI_IP> robot:=fr3 arm_id:=pingvin_1 legacy_franka_ros:=false __ns:=pingvin_1 load_gripper:=true controller:=cartesian_impedance_controller` 

Parameters:

* `robot_ip`: FCI IP
* `load_gripper` Should a Franka hand be mounted on the flange?
* `robot`: Robot type. Possible values: [panda, fr3]
* `arm_id`: Name of the robot to spawn, usually same as robot type
* `robot_ns`: (Optional) name-space of the robot when using multiple robots. If using this option, set arm_id accordingly.
* `legacy_franka_ros`:  true for libfranka versions 0.8 or lower 
* `controller`: Which controller should be started? One of `{cartesian_impedance,joint_impedance,joint_velocity}_controller`



## Launch in the simulation

`roslaunch ijs_controllers sim.launch controller:=cartesian_impedance_controller`

Parameters are same as above.



## Example usage in Python
All controllers expose a `command` topic. The message can be sent to it using standard ROS publisher and a helper function given by the following exemplary code.

```python
import rospy
from robot_module_msgs.msg import ImpedanceParameters, CartesianCommand
...
self.cartesian_command_publisher = rospy.Publisher('/panda_1/cartesian_impedance_controller/command', CartesianCommand, queue_size=1)
...
```

```python
def GoTo_X(self, x, xdot, trq, **kwargs):
    """GoTo_X Update task position and orientation for task controller and wait for time t
        Inputs:
        x       task position,quaternion (1 x 7) (in robot CS)
        xdot    task twist (6 x 1) (in robot CS)
        trq     additional wrench (6 x 1) (in robot CS)
            
        Options:
        Kp  position stiffness
        Kr  orientation stiffness
        R   stiffness rotation
        D   damping factor
    """

    kwargs.setdefault('R',np.eye(3, dtype= float))
    kwargs.setdefault('D',2)
    kwargs.setdefault('Kp',np.ones(3,dtype=float)*2000)
    kwargs.setdefault('Kr',np.ones(3, dtype=float)* 50)
    R = kwargs['R']
    D = kwargs['D']
    Kp = kwargs['Kp']
    Kr = kwargs['Kr']
    
    cmd_msg = CartesianCommand()
    
    cmd_msg.pose.position.x = x[0]
    cmd_msg.pose.position.y = x[1]
    cmd_msg.pose.position.z = x[2]
    cmd_msg.pose.orientation.w = x[3]
    cmd_msg.pose.orientation.x = x[4]
    cmd_msg.pose.orientation.y = x[5]
    cmd_msg.pose.orientation.z = x[6]
    
    cmd_msg.vel.linear.x = xdot[0]
    cmd_msg.vel.linear.y = xdot[1]
    cmd_msg.vel.linear.z = xdot[2]
    cmd_msg.vel.angular.x = xdot[3]
    cmd_msg.vel.angular.y = xdot[4]
    cmd_msg.vel.angular.z = xdot[5]
    
    cmd_msg.ft.force.x = trq[0]
    cmd_msg.ft.force.y = trq[1]
    cmd_msg.ft.force.z = trq[2]
    
    cmd_msg.ft.torque.x = trq[3]
    cmd_msg.ft.torque.y = trq[4]
    cmd_msg.ft.torque.z = trq[5]
    
    # Stiffness matrix
    trM = np.diag(Kp)
    rotM = np.diag(Kr)
    trK = R * trM * np.transpose(R)
    rotK = rotM
    
    # Damping
    trD = R * 2 * np.sqrt(trM)*np.transpose(R)
    rotD = D*np.sqrt(rotM)
    
    cmd_msg.impedance = ImpedanceParameters()
    cmd_msg.impedance.n = 9
    cmd_msg.impedance.k = np.concatenate([np.reshape(trK, (9,1)), np.reshape(rotK, (9,1))])
    cmd_msg.impedance.d = np.concatenate([np.reshape(trD, (9,1)), np.reshape(rotD, (9,1))])
    
    # Publish
    self.cartesian_command_publisher.publish(cmd_msg)
```

Note that, the `command` topic expects that the trajectory points are dent in equal time steps. This can be achieved  in the following way:

```python
def ExecuteTrajectory(self, trajectory, wait):
    ...
    for point in trajectory
        ...
        self.GetState()
        self.GoTo_X(point.x, point.xdot, point.trq)
        
        while (lapsed_time - self.last_control_time) < wait:
            ...
        self.last_control_time= tm()
        
        ...
```

