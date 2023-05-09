#!/home/soroush/anaconda3/envs/obb/bin/python
import numpy as np
import rospy
import numpy
from geometry_msgs.msg import Twist


class FeedForwardController(object):
    def __init__(self, linear_vel_path, rotational_vel_path):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lin_vel_cmds, self.rot_vel_cmds = self._load_command_velocities(linear_vel_path, rotational_vel_path)
        self.last_command_idx = 0

    @staticmethod
    def _load_command_velocities(linear_vel_path, rotational_vel_path):
        linear_vel_commands = np.load(linear_vel_path)
        rotational_vel_commands = np.load(rotational_vel_path)
        return linear_vel_commands, rotational_vel_commands

    def _create_twist_msg(self, index):
        lin_vel_cmd = self.lin_vel_cmds[index]
        rot_vel_cmd = self.rot_vel_cmds[index]
        twist = Twist()
        twist.linear.x = lin_vel_cmd
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = rot_vel_cmd
        return twist

    def pub_vel_command(self):
        index = min(self.last_command_idx, self.lin_vel_cmds.shape[0] - 1)
        twist = self._create_twist_msg(index)
        self.publisher.publish(twist)
        self.last_command_idx += 1
        return


if __name__ == '__main__':
    try:
        rotational_velocity_commands_path = "./rotational_vel.npy"
        liner_velocity_commands_path = "./linear_vel.npy"
        rospy.init_node('husky_controller', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        controller = FeedForwardController(liner_velocity_commands_path, rotational_velocity_commands_path)
        while not rospy.is_shutdown():
            # pub.publish(hello_str)
            controller.pub_vel_command()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
