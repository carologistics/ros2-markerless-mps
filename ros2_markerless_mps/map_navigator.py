import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from nav2_simple_commander.robot_navigator import BasicNavigator
class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')
        self.corner_poses = [
            [-0.5, 0.5, 0.0],         # Start from the origin
            [-0.5, 5.5, 1.57],        # Top-right corner
            [-4.5, 5.5, 3.14],       # Top-left corner
            [-4.5, 1.5, -1.57],      # Bottom-left corner
            #[0.0, 0.0, 0.0]          # Return to the origin
        ]
        self.current_corner = 0
        self.nav = BasicNavigator()

        

    def send_goal(self, pose):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = pose[0]
        goal_msg.pose.position.y = pose[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.z = pose[2]
        self.nav.goToPose(goal_msg)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

       
                
        

    def navigate_to_next_corner(self):
        if self.current_corner >= len(self.corner_poses):
            self.get_logger().info('Finished navigating all corners.')
            return
        self.get_logger().info(f'Navigating to corner {self.current_corner+1}')
        self.send_goal(self.corner_poses[self.current_corner])
        

        self.current_corner += 1

    

def main(args=None):
    rclpy.init(args=args)
    navigator = MapNavigator()
    navigator.navigate_to_next_corner()
    navigator.navigate_to_next_corner()
    navigator.navigate_to_next_corner()
    navigator.navigate_to_next_corner()
    
    while rclpy.ok():
        rclpy.spin_once(navigator)
        if navigator.current_corner >= len(navigator.corner_poses):
            break

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
