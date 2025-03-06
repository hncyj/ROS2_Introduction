import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, node_name:str, name:str, age:int) -> None:
        super().__init__(node_name)
        self.name = name
        self.age = age
        
    def eat(self, food:str):
        self.get_logger().info(f"my name is {self.name}, {self.age} years old, and eating {food} now.")
        

def main():
    rclpy.init()
    node = PersonNode('person_node', 'chenyinjie', 18)
    node.eat('apple')
    rclpy.spin(node)
    rclpy.shutdown()