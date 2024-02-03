import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
from math import fabs

DEAD_ZONE = 0.1

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joy_node')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.init_joystick()
        self.target_id = 0
        self.get_logger().info("joy node is running...")

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()

    def play(self):
        joysticks = {}
        done = False

        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True  # Flag that we are done so we exit this loop.

                # Handle hotplugging
                if event.type == pygame.JOYDEVICEADDED:
                    # This event will be generated when the program starts for every
                    # joystick, filling up the list without needing to create them manually.
                    joy = pygame.joystick.Joystick(event.device_index)
                    id = joy.get_instance_id()
                    joysticks[id] = joy
                    self.get_logger().info(f"Joystick {id} connected")

                    if id == self.target_id:
                        self.joystick = joy
                        self.get_logger().info("Started Publishing Joystick Message")

                if event.type == pygame.JOYDEVICEREMOVED:
                    id = event.instance_id
                    del joysticks[id]
                    self.get_logger().info(f"Joystick {id} disconnected")

                    if id == self.target_id:
                        self.target_id += 1
                        self.get_logger().info("Stopped Publishing Joystick Message")

            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                continue
            
            for joystick in joysticks.values():
                jid = joystick.get_instance_id()
                
                if jid == self.target_id:
                    try:
                        joy_msg = Joy()
                        pygame.event.pump()
                        joy_msg.header.stamp = self.get_clock().now().to_msg()

                        # Map buttons
                        joy_msg.buttons = [
                            joystick.get_button(0),  # Cross Button
                            joystick.get_button(1),  # Circle Button
                            joystick.get_button(2),  # Square Button
                            joystick.get_button(3),  # Triangle Button
                            joystick.get_button(4),  # Left Bumper
                            joystick.get_button(5),  # Right Bumper
                            joystick.get_button(9),  # Option Button
                            joystick.get_button(8),  # Share Button
                            joystick.get_button(7),  # Start Button
                            joystick.get_button(10), # Power Button
                            joystick.get_button(11), # Button Stick Left
                            joystick.get_button(12)  # Button Stick Right
                        ]

                        # Map axes
                        joy_msg.axes = [
                            joystick.get_axis(0) if fabs(joystick.get_axis(0)) > DEAD_ZONE else 0.0,  # Left/Right Axis stick left
                            joystick.get_axis(1) if fabs(joystick.get_axis(1)) > DEAD_ZONE else 0.0,  # Up/Down Axis stick left
                            joystick.get_axis(3) if fabs(joystick.get_axis(3)) > DEAD_ZONE else 0.0,  # Left/Right Axis stick right
                            joystick.get_axis(4) if fabs(joystick.get_axis(4)) > DEAD_ZONE else 0.0,  # Up/Down Axis stick right
                            joystick.get_axis(2) if fabs(joystick.get_axis(2)) > DEAD_ZONE else 0.0,  # Left Trigger
                            joystick.get_axis(5) if fabs(joystick.get_axis(5)) > DEAD_ZONE else 0.0,  # Right Trigger
                            float(self.joystick.get_hat(0)[0]),                                       # Left/Right hat
                            float(self.joystick.get_hat(0)[1])                                        # Down/Up hat
                        ]

                        self.publisher_.publish(joy_msg)

                    except pygame.error as e:
                        self.get_logger().error(f"Pygame error: {e}")

                    except KeyboardInterrupt:
                        self.get_logger().info("Shutting down...")
                        pygame.quit()
                        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    joy = JoystickNode()
    joy.play()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
