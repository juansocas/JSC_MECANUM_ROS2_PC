#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):

    #! JSC: Creamos constructor
    def __init__(self):
        #? JSC: Nombre del nodo que va a aparecer en los graphs
        super().__init__("fist_node")
        self.counter_ = 0
        #! JSC: Creamos un timer para publicar en el terminal
        self.create_timer(1.0,self.timwe_callback)

    def timwe_callback(self):
        #! JSC: Coger y manda informacion del log
        self.get_logger().info("HELLO " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    #! JSC: Inicalizamos ROS2 comunication con los args de arriba
    rclpy.init(args=args)

    #! JSC: Creamos el nodo

    node = MyNode()

    #! JSC: Conseguimos que el nodo se siga manteniedo activo hasta hacer Cntrl + C
    rclpy.spin(node)



    #! JSC: Cerramos la comunicacion ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
