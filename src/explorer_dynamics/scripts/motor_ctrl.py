#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time

from robust_serial import write_order, Order, write_i8, write_i16, read_i16, read_i32, read_i8
from robust_serial.utils import open_serial_port

rospy.init_node('listener', anonymous=True)

#######################################################################
##                Setup Communication with Arduino                   ##
#######################################################################
serial_file = None
BAUDRATE = 115200  # Communication with the Arduino

try:
    # Open serial port (for communication with Arduino)
    serial_file = open_serial_port(baudrate=BAUDRATE)

except Exception as e:
    rospy.loginfo('Could not open serial file')
    raise e

is_connected = False
# Initialize communication with Arduino

while not is_connected:
    print('trying')
    rospy.loginfo("Trying to establish connection to Arduino...")
    write_order(serial_file, Order.HELLO)
    bytes_array = bytearray(serial_file.read(1))
    if not bytes_array:
        time.sleep(2)
        continue
    byte = bytes_array[0]
    if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
        is_connected = True

time.sleep(2)

c=1
while (c!=b''):
    c = serial_file.read(1)

rospy.loginfo("*** Connection with Arduino established ***")

#######################################################################
##                 Subscribe to topic 23_dynamic                     ##
#######################################################################
def motor_crtl(data):
    rospy.loginfo(rospy.get_caller_id() + "motor_crtl: I heard %s", data.data)
    if data.data == 'f':
        # forward
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, 100) #value right motor
        write_i8(serial_file, 100) #value left motor
        time.sleep(2)
        write_order(serial_file, Order.STOP)
    elif data.data == 'b':
        # backward
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -100) #value right motor
        write_i8(serial_file, -100) #value left motor
        time.sleep(2)
        write_order(serial_file, Order.STOP)
    elif data.data == 'r':
        # spin right
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, -100) #value right motor
        write_i8(serial_file, 100) #value left motor
        time.sleep(2)
        write_order(serial_file, Order.STOP)
    elif data.data == 'l':
        # spin left
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, 100) #value right motor
        write_i8(serial_file, -100) #value left motor
        time.sleep(2)
        write_order(serial_file, Order.STOP)
    
rospy.Subscriber("dynamic_23", String, motor_crtl)
rospy.spin()