import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class DigitController(Node):

    def __init__(self):
        super().__init__('digit_controller')

	# eti 2 stroki sozdayut parametri dlya 0 i 3
        self.declare_parameter('turtle_name', 'turtle1')
        self.declare_parameter('digit', 0)

	# zdes  znacheniya zabirayut
        self.turtle_name = self.get_parameter('turtle_name').value
        self.digit = self.get_parameter('digit').value
# marshruti deystviy dlya 0 i 3
        routes = {
            0: [
                ("move", 3.0),
                ("turn", 0.0),
                ("move", 1.0),
                ("turn", -math.pi / 2),
                ("move", 3.0),
                ("turn", math.pi),
                ("move", 1.0),
            ],

            3: [
                ("move", 1.0),
                ("turn", math.pi / 2),
                ("move", 2.0),
                ("turn", math.pi),
                ("move", 1.0),
                ("turn", 0.0),
                ("move", 1.0),
                ("turn", math.pi / 2),
                ("move", 2.0),
                ("turn", math.pi),
                ("move", 1.0),
            ],
        }
# stroka vibiraet nugniy marshrut
        self.actions = routes[self.digit]
# nachinaem s 1 deystviya marshruta
        self.action_index = 0
# dlya sbora dannih pozhe
        self.start_x = None
        self.start_y = None
# otpravlyaet komandy skorosti
        self.publisher = self.create_publisher(
            Twist,
            f'/{self.turtle_name}/cmd_vel',
            10
        )
# poluchaet pogenie cherepahi
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtle_name}/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
# proverka zakonchilsa li marshrut
        if self.action_index >= len(self.actions):
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.publisher.publish(velocity)
            return
# berert zhachenie actual deystviya
        action = self.actions[self.action_index]

        action_type = action[0]
        value = action[1]
# proveryaet deystvie - move?
        if action_type == "move":
# zabiraet nachalnoe znachenie polozeniya, esli perviy hod
            if self.start_x is None:
                self.start_x = msg.x
                self.start_y = msg.y
# raschet distance kotoruyu nuzno proyti
            distance = math.sqrt(
                (msg.x - self.start_x) ** 2 +
                (msg.y - self.start_y) ** 2
            )

            velocity = Twist()
# esli mense nuznogo, to prodolzaem dvizenie
            if distance < value:
                velocity.linear.x = 1.0
                velocity.angular.z = 0.0
# inache stop + next move i onulenie nachalnoy koordinaty i vzatie novoy
            else:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.0

                self.action_index += 1

                self.start_x = None
                self.start_y = None
# otpravka
            self.publisher.publish(velocity)
# esli deystvie ne move
        elif action_type == "turn":
# nuznoe znachenie
            target_theta = value
# raznica tekushego i nuznogo znacheniya v radianah
            difference = math.atan2(
                math.sin(target_theta - msg.theta),
                math.cos(target_theta - msg.theta)
            )

            velocity = Twist()
# prodolzaem izmenenie esli znacheniya silno raznyatsa
            if abs(difference) > 0.05:
                velocity.linear.x = 0.0

                if difference > 0:
                    velocity.angular.z = 1.0
                else:
                    velocity.angular.z = -1.0
# inache stop
            else:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.0
# perehod k next move
                self.action_index += 1

            self.publisher.publish(velocity)


def main(args=None):
# zapuskaet ros2 dlya python
    rclpy.init(args=args)
# sozdat moy kontroller
    node = DigitController()
# ostavlyaet uzel rabotat i wait new msg
    rclpy.spin(node)
# zaversayut rabotu
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
