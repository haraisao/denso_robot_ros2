#
#
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
import numpy as np

from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
from denso_robot_core_interfaces.srv import GetMode

try:
  from rc8_client import Rc8Client
except:
  from .rc8_client import Rc8Client

ACTION_NAME = '/denso_joint_trajectory_controller/follow_joint_trajectory'

class CobottControlServer(Node):
    def __init__(self):
        super().__init__('cobotta_ctrl_server')
        self.get_logger().info("=== Init BCAP Cobotta Control Server")
        self.init_parameters()
        self.ip_addr = self.get_parameter("ip_address").value
        self.ESP = self.get_parameter("restruct_esp1").value

        self.get_mode_client = self.create_client(GetMode, "/cobotta/GetMode")
        res_ =self.get_mode_client.wait_for_service(timeout_sec=5.0)
        if not res_ :
            self.get_logger().info('service /cobotta/GetMode not available')
        
        #
        #
        self._action_server = ActionServer(self, FollowJointTrajectory, ACTION_NAME, 
               execute_callback = self.execute_callback,
               goal_callback = self.goal_callback,
               cancel_callback = self.cancel_callback)
       
        #
        # Connect to RC8
        self.get_logger().info("===> RC8: %s" % self.ip_addr)
        self.client = Rc8Client(self.ip_addr)
        self.client.connect()
        if self.client.is_connected() :
            self.get_logger().info("===>Start bcap controller")
        else:
            self.get_logger().info("===>fail to connect to RC8 controller")

    def init_parameters(self):
        self.declare_parameter("ip_address", "192.168.0.1")
        self.declare_parameter("restruct_esp1", 0.17)
        return

    def connect(self):
        self.client.connect()
        return

    def disconnect(self):
        self.client.disconnect()
        return

    def get_control_mode(self):
        res_  = self.get_mode_client.wait_for_service(timeout_sec=1.0)
        if not res_ : return None
        req = GetMode.Request()
        self.future = self.get_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().mode

    def execute_callback(self, goal):
        if not self.get_control_mode() :
            self.get_logger().info('====> Execute goal')
            restruct_trj_ = self.get_joint_trajectory(goal.request.trajectory, deg=True, restruct=True)
            i=0
            for x in restruct_trj_:
                self.get_logger().info("====> {}: {}".format(i, x))
                i += 1 

            if self.client.is_connected():
                self.client.take_arm()
                self.client.move_joint_trajectory(restruct_trj_, wait=True)
                self.client.give_arm()
            else:
                self.get_logger().info("Fail to move")
        else:
            self.get_logger().info("Currnt mode is not normal...")

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
            self.get_logger().info("Invaid arguments: {}".format(trj.points))
            import traceback
            traceback.print_exc()
            return None

    def restruct_waypoints(self, trj):
        dt = np.array(trj[1])  - np.array(trj[0])
        dt = dt/np.linalg.norm(dt)
        res = [trj[0]]

        for i in range(len(trj) - 2):
            dt_tmp = np.array(trj[i+2]) - np.array(trj[i+1])
            dt_tmp = dt_tmp/np.linalg.norm(dt_tmp)
            if np.linalg.norm(dt - dt_tmp) > self.ESP :
                self.get_logger().info("new waipoint: {}".format(np.linalg.norm(dt - dt_tmp)))
                res.append(trj[i+1])
                dt = dt_tmp
        res.append(trj[-1])
        #return self.skip_close_points(res)
        return res

    def skip_close_points(self, trj, ESP=0.01):
        res=[trj[0]]
        p0=self.client.convert_j_to_p(trj[0])
        for i in range(1, len(trj)-1):
            p1=self.client.convert_j_to_p(trj[i])
            self.get_logger().info("POS ====> {}: {}".format(i, p1))
            if np.linalg.norm(p1[:3] - p0[:3]) > ESP:
                res.append(trj[i])
                p0=p1
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
