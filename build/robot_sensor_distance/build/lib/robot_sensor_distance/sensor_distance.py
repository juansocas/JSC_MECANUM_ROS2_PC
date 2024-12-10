
import sys
import crcmod.predefined 
from std_msgs.msg import Float32
import serial
import serial.tools.list_ports
import rclpy
from rclpy.node import Node



class SensorDistanceNode(Node):

    #! JSC: Creamos constructor
    def __init__(self):
        #? JSC: Nombre del nodo que va a aparecer en los graphs
        super().__init__("distancia")
        self.counter_ = 0
        #! JSC: Creamos un timer para publicar en el terminal
        self.create_timer(1.0,self.timwe_callback)

    def timwe_callback(self):
        #! JSC: Coger y manda informacion del log
        self.get_logger().info("HELLO " + str(self.counter_))
        self.counter_ += 1



def findEvo():
    #! Find Live Ports, return port name if found, NULL if not
    print('Scanning all live ports on this PC')
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print p # This causes each port's information to be printed out.
        if "5740" in p[2]:
            print('Evo found on port ' + p[0])
            return p[0]
    return 'NULL'

def openEvo(portname):
    print('Attempting to open port...')
    #! Open the Evo and catch any exceptions thrown by the OS
    print('EL pueto usado es el: ' + portname)
    evo = serial.Serial(portname, baudrate=115200, timeout=2)
    #! Send the command "Binary mode"
    set_bin = (0x00, 0x11, 0x02, 0x4C)
    #! Flush in the buffer
    evo.flushInput()
    #! Write the binary command to the Evo
    evo.write(set_bin)
    #! Flush out the buffer
    evo.flushOutput()
    print('Serial port opened')
    return evo

""" 
#! JSC: FUNCION QUE CAPTA LA DISTANCIA Y LA RETORNA 

#! @param evo_serial: El puerto
#! @return: Distancia -> Float
#! @raise keyError: raises an exception
"""
def get_evo_range(evo_serial):
    crc8_fn = crcmod.predefined.mkPredefinedCrcFun('crc-8')
    #! Read one byte
    data = evo_serial.read(1)
    if data == b'T':
        # After T read 3 bytes
        frame = data + evo_serial.read(3)
        if frame[3] == crc8_fn(frame[0:3]):
            # Convert binary frame to decimal in shifting by 8 the frame
            rng = frame[1] << 8
            rng = rng | (frame[2] & 0xFF)
        else:
            return "CRC mismatch. Check connection or make sure only one progam access the sensor port."
    # Check special cases (limit values)
    else:
        return "Wating for frame header"

    # Checking error codes
    if rng == 65535: # Sensor measuring above its maximum limit
        dec_out = float('inf')
    elif rng == 1: # Sensor not able to measure
        dec_out = float('nan')
    elif rng == 0: # Sensor detecting object below minimum range
        dec_out = -float('inf')
    else:
        # Convert frame in meters
        dec_out = rng / 1000.0
    return dec_out



def main(args=None):
    #! JSC: Inicalizamos ROS2 comunication con los args de arriba
    rclpy.init(args=args)

    #! JSC: Creamos el nodo

    node = SensorDistanceNode()

    #! JSC: Conseguimos que el nodo se siga manteniedo activo hasta hacer Cntrl + C
    rclpy.spin(node)


    #! JSC: Cerramos la comunicacion ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()