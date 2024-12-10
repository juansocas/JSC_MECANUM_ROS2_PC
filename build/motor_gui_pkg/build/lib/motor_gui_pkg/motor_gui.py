import rclpy
from rclpy.node import Node
import time
from tkinter import *
import math
import serial  # Importar para la comunicación por puerto serie

class MotorGui(Node):

    def __init__(self):
        super().__init__('motor_gui')

        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)  # Configurar el puerto serie

        self.tk = Tk()
        self.tk.title("Serial Motor GUI")
        self.tk.geometry("600x400") 
        root = Frame(self.tk)
        root.pack(fill=BOTH, expand=True)

        Label(root, text="Serial Motor GUI").pack()

        m1_frame = Frame(root)
        m1_frame.pack(fill=X)
        Label(m1_frame, text="Motor 1").pack(side=LEFT)
        self.m1 = Scale(m1_frame, from_=-255, to=255, orient=HORIZONTAL)
        self.m1.pack(side=LEFT, fill=X, expand=True)

        m2_frame = Frame(root)
        m2_frame.pack(fill=X)
        Label(m2_frame, text="Motor 2").pack(side=LEFT)
        self.m2 = Scale(m2_frame, from_=-255, to=255, resolution=1, orient=HORIZONTAL)
        self.m2.pack(side=LEFT, fill=X, expand=True)

        m3_frame = Frame(root)
        m3_frame.pack(fill=X)
        Label(m3_frame, text="Motor 3").pack(side=LEFT)
        self.m3 = Scale(m3_frame, from_=-255, to=255, resolution=1, orient=HORIZONTAL)
        self.m3.pack(side=LEFT, fill=X, expand=True)

        m4_frame = Frame(root)
        m4_frame.pack(fill=X)
        Label(m4_frame, text="Motor 4").pack(side=LEFT)
        self.m4 = Scale(m4_frame, from_=-255, to=255, resolution=1, orient=HORIZONTAL)
        self.m4.pack(side=LEFT, fill=X, expand=True)

        motor_btns_frame = Frame(root)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_motor_once).pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Motor', command=self.stop_motors).pack(side=LEFT)

    def send_motor_once(self):
        # Obtener las velocidades de los cuatro motores
        speed_motor1 = self.m1.get()
        speed_motor2 = self.m2.get()
        speed_motor3 = self.m3.get()
        speed_motor4 = self.m4.get()

        # Enviar los datos en el formato especificado por el usuario
        command = f"M:{speed_motor1}:{speed_motor2}:{speed_motor3}:{speed_motor4}\n"
        self.serial_port.write(command.encode())
        print(f"Sent command: {command}")

    def stop_motors(self):
        # Enviar comando para detener los motores
        command = "M:0:0:0:0\n"
        self.serial_port.write(command.encode())
        print(f"Sent command: {command}")

    def update(self):
        self.tk.update()


def main(args=None):
    rclpy.init(args=args)

    motor_gui = MotorGui()

    rate = motor_gui.create_rate(20)    
    while rclpy.ok():
        rclpy.spin_once(motor_gui)
        motor_gui.update()

    motor_gui.destroy_node()
    rclpy.shutdown()
