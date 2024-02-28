import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import AutoPath
from custom_interfaces.srv import RelativeLocation
from geometry_msgs.msg import Twist
import math

class RelativeLocationServer(Node):
    def __init__(self):
        super().__init__('relative_location_server')

        # Setup client, server and publisher

        self.server = ActionServer(self, AutoPath, 'autopath_as', self.execute_callback)
        self.relative_location_client = self.create_client(RelativeLocation, 'get_relative_location')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        # Configuration

        self.stationary_turn_threshold = 55  # Turn treshold while stationary
        self.turn_deadzone = 5  # Turn treshold while moving towards target
        self.reverse_treshold_angle = 120 # Target is considered behind us if turn_degrees is higher than this number
        self.reverse_treshold_distance = 1.5 # We reverse if target is behind us and within this range in meters.
        
    # This happens when we get a goal from client/terminal

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        self.feedback_msg = AutoPath.Feedback()
        
        # Define goal

        x = goal_handle.request.x
        y = goal_handle.request.y
        endpoint = goal_handle.request.endpoint
        
        # Start loop

        try:
            while rclpy.ok():

                # Request distance/angle from relative_location_service, only move if we get an actual response.

                response = await self.call_relative_location_service(x, y)
                if response:
                    self.turn_degrees = response.angle_to_rotate
                    self.distance = response.distance
                    self.feedback_msg.degrees = self.turn_degrees
                    self.feedback_msg.distance = self.distance
                    goal_handle.publish_feedback(self.feedback_msg)

                    result = AutoPath.Result()
                    result.success = False

                    
                    # Slow down if we are arriving at endpoint
                    if self.distance < 0.2 and endpoint:
                        velocity_multiplier = 0.2

                    elif self.distance < 0.5 and endpoint:
                        velocity_multiplier = 0.5

                    elif self.distance < 1.0 and endpoint:
                        velocity_multiplier = 0.8
                    
                    else:
                        velocity_multiplier = 1.0
                    
                    # Check if we should reverse into target
                    if abs(self.turn_degrees) > self.reverse_treshold_angle and self.distance < self.reverse_treshold_distance:
                        reverse = True
                    else:
                        reverse = False                  

                    ##################### REVERSING ##########################################

                    if reverse == True:

                        # If we are NOT aimed at target, we rotate before reversing.
                        if self.turn_degrees <= 175 and self.turn_degrees > self.reverse_treshold_angle:
                            self.twist.angular.z = -0.4
                            self.twist.linear.x = 0.0
                            self.cmd_vel_publisher.publish(self.twist)

                        elif self.turn_degrees >= -175 and self.turn_degrees < (self.reverse_treshold_angle * -1):
                            self.twist.angular.z = 0.4  
                            self.twist.linear.x = 0.0
                            self.cmd_vel_publisher.publish(self.twist)

                        # If we are aimed precicly at the target we dont turn                            
                        elif abs(self.turn_degrees) >= 178:
                            self.twist.angular.z = 0.0 
                            self.twist.linear.x = -0.3
                            self.cmd_vel_publisher.publish(self.twist)   

                        # If we are roughly aimed at target, we turn a bit while reversing.
                        elif self.turn_degrees <= -175 and self.turn_degrees < (self.reverse_treshold_angle * -1):
                            self.twist.angular.z = 0.2
                            self.twist.linear.x = -0.3
                            self.cmd_vel_publisher.publish(self.twist)

                        elif self.turn_degrees >= 175 and self.turn_degrees > self.reverse_treshold_angle:
                            self.twist.angular.z = -0.2 
                            self.twist.linear.x = -0.3
                            self.cmd_vel_publisher.publish(self.twist)   

                    ##################### DRIVING FORWARD ##########################################

                    elif reverse == False:
                        #Check wich way to rotate
                        if self.turn_degrees > 0:
                            turnDirection = 1
                        elif self.turn_degrees <= 0:
                            turnDirection = -1   

                        # If we are close to the target and target is in front
                        if abs(self.turn_degrees) > (30) and self.distance <= 1.0:
                            self.twist.angular.z = 0.7 * turnDirection  
                            self.twist.linear.x = 0.0
                            self.cmd_vel_publisher.publish(self.twist)
                            print("1")
                        # If we are close to the target, but target is behind

                        # If we are NOT aimed at target, we turn around before driving.
                        elif abs(self.turn_degrees) >= (self.stationary_turn_threshold):
                            
                            self.twist.angular.z = 0.7 * turnDirection  
                            self.twist.linear.x = 0.0
                            self.cmd_vel_publisher.publish(self.twist)
            
                        # If we are aimed precicly at the target we dont turn
                        elif abs(self.turn_degrees) < (self.turn_deadzone):
                            self.twist.angular.z = 0.0
                            self.twist.linear.x = 0.5  * velocity_multiplier
                            self.cmd_vel_publisher.publish(self.twist)
                
                        # If we are roughly aimed at target, we turn a bit.
                        else:
                            self.twist.angular.z = 0.4 * turnDirection  
                            self.twist.linear.x = 0.5  * velocity_multiplier
                            self.cmd_vel_publisher.publish(self.twist)
            

                    # Arrived at target
                    if self.distance < 0.1:
                        
                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.0
                        self.cmd_vel_publisher.publish(self.twist)
                        self.get_logger().info('Goal executed successfully')
                        goal_handle.succeed()
                        
                        result.success = True
                        return result

        except Exception as e:
            self.get_logger().error(f"Failed to execute action: {str(e)}")
            goal_handle.abort()
            result = AutoPath.Result()
            return result

    async def call_relative_location_service(self, x, y):
        srv_req = RelativeLocation.Request()
        srv_req.x = x
        srv_req.y = y

        try:
            response = await self.relative_location_client.call_async(srv_req)
            return response
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    relative_location_server = RelativeLocationServer()
    rclpy.spin(relative_location_server)
    relative_location_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
