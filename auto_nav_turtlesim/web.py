#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8


class WebSym(Node):
    def __init__(self):
        super().__init__("WebSym")
        self.get_logger().info('Turtle controller is ready ...')
        self.pub =self.create_publisher(Float32MultiArray, "target_coordinates",10)
        self.pub2 = self.create_publisher(Int8,"target_type",10)
        print("Target type:")
        self.target_type=int(input())
        print ("Target x: ")
        self.target_x=float(input())
        print ("Target y: ")
        self.target_y=float(input())
        msg2 = Int8()
        msg2.data=self.target_type
        msg = Float32MultiArray()
        msg.data = [self.target_x,self.target_y]
        self.pub2.publish(msg2)
        self.pub.publish(msg)
        print ("Target type and coordinates published")

def main(args=None):
  rclpy.init(args=args)
  nodeh = WebSym()
  try: rclpy.spin(nodeh)
  except Exception as error: print(error)
  except KeyboardInterrupt: print("Node Terminated!")
  
if __name__=="__main__":
  main()