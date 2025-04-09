#
#
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
import numpy as np

from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory

try:
  from rc8_client import Rc8Client
except:
  from .rc8_client import Rc8Client

ACTION_NAME = '/denso_joint_trajectory_controller/follow_joint_trajectory'

class CobottControlServer(Node):
    def __init__(self):
        super().__init__('cobotta_ctrl_server')
        self.get_logger().info("=== Init BCAP Cobotta Control Server")
        self._action_server = ActionServer(self, FollowJointTrajectory, ACTION_NAME, 
               execute_callback = self.execute_callback,
               goal_callback = self.goal_callback,
               cancel_callback = self.cancel_callback)
        ip_addr = "192.168.0.1"
        self.client = Rc8Client(ip_addr)
        self.client.connect()
        self.get_logger().info("===>Start bcap controller")

    def connect(self):
        self.client.connect()
        #self.client.take_arm()
        return

    def disconnect(self):
        self.client.disconnect()
        return

    def execute_callback(self, goal):
        self.get_logger().info('Execute goal')
        restruct_trj_ = self.get_joint_trajectory(goal.request.trajectory, deg=True, restruct=True)
        for x in restruct_trj_:
            print(x)

        if self.client.is_connected():
          self.client.take_arm()
          self.client.move_joint_trajectory(restruct_trj_, wait=True)
          self.client.give_arm()
        else:
          self.get_logger().info("Fail to move")

        goal.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

    def goal_callback(self, goal):
        self.get_logger().info("Received goal request")
    
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def get_joint_trajectory(self, trj, deg=False, restruct=False):
        trj_=[]
        try:
            #trj_ = trj
            for p in trj.points:
                if deg:
                    trj_.append(np.rad2deg(p.positions))
                else:
                    trj_.append(p.positions)
            if restruct:
                return self.restruct_waypoints(trj_)
            else:
                return trj_
        except:
            print("Invalid argument")
            return None

    def restruct_waypoints(self, trj, ESP=0.01):
        dt = np.array(trj[1])  - np.array(trj[0])
        dt = dt/np.linalg.norm(dt)
        res = [trj[0]]

        for i in range(len(trj) - 2):
            dt_tmp = np.array(trj[i+2]) - np.array(trj[i+1])
            dt_tmp = dt_tmp/np.linalg.norm(dt_tmp)
            if np.linalg.norm(dt - dt_tmp) > ESP :
                res.append(trj[i+1])
                dt = dt_tmp
        res.append(trj[-1])
        return res


def main(args=None):
    rclpy.init(args=args)

    server_ = CobottControlServer()
    rclpy.spin(server_)

    server_.disconnect()
    server_.destroy_node()

if __name__ == '__main__':
    main()
