from typing import List
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory


class GoStraightAhead(Node):
    def __init__(self) -> None:
        super().__init__("control_cmd_subscriber")

        self.pub = self.create_publisher(
            AckermannControlCommand,
            "/control/trajectory_follower/control_cmd",
            10,
        )

        self.sub = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/trajectory",
            self.listener_callback,
            10,
        )
        self.sub

    def listener_callback(self, msg: Trajectory):
        print(msg)
        cmd = AckermannControlCommand()
        # ひたすら前進するように
        cmd.longitudinal.speed = 10000.0
        cmd.longitudinal.acceleration = 10000.0
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GoStraightAhead()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
