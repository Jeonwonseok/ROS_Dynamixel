#!/usr/bin/env python

import rospy
import os
import ctypes
from beginner_tutorials.msg import Data

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

#os.sys.path.append('../dynamixel_functions_py')             # Path setting

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36

# Data Byte Length
LEN_MX_GOAL_POSITION        = 2
LEN_MX_PRESENT_POSITION     = 2

# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2
DXL3_ID                      = 3                             # Dynamixel ID: 3
DXL4_ID                      = 4                             # Dynamixel ID: 4
DXL5_ID                      = 5                             # Dynamixel ID: 5
DXL6_ID                      = 6                             # Dynamixel ID: 6
DXL7_ID                      = 7                             # Dynamixel ID: 7
DXL8_ID                      = 8                             # Dynamixel ID: 8
DXL9_ID                      = 9                             # Dynamixel ID: 9
DXL10_ID                      = 10                             # Dynamixel ID: 10
DXL11_ID                      = 11                             # Dynamixel ID: 11
DXL12_ID                      = 12                             # Dynamixel ID: 12
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 1500                           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2500                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 3                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

# Initialize Groupsyncwrite instance
group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

dxl_comm_result = COMM_TX_FAIL                              # Communication result

dxl_present_position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def callback(data):
	rospy.loginfo(data.data)
	dxl_poscon(data.data)

def dxl_poscon(goal_position):
	# Add Dynamixels goal position value to the Syncwrite storage
	for i in xrange(12):
		dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, i+1, goal_position[i], LEN_MX_GOAL_POSITION)).value
		print(dxl_addparam_result)
		if dxl_addparam_result != 1:
		    print(dxl_addparam_result)
		    print("[ID:%03d] groupSyncWrite addparam failed" % (i+1))
#		    quit()

    # Syncwrite goal position
	dynamixel.groupSyncWriteTxPacket(group_num)
	if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
		dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

	# Clear syncwrite parameter storage
	dynamixel.groupSyncWriteClearParam(group_num)

	while 1:
		# Read present position
		for j in xrange(12):
			dxl_present_position[j] = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, j+1, ADDR_MX_PRESENT_POSITION)
			if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
				dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
			elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
				dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

			print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (j+1, goal_position[j], dxl_present_position[j]))

		if not (abs(goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[4] - dxl_present_position[4]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[5] - dxl_present_position[5]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[6] - dxl_present_position[6]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[7] - dxl_present_position[7]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[8] - dxl_present_position[8]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[9] - dxl_present_position[9]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[10] - dxl_present_position[10]) > DXL_MOVING_STATUS_THRESHOLD) or \
	(abs(goal_position[11] - dxl_present_position[11]) > DXL_MOVING_STATUS_THRESHOLD):
			break

def dxl_init():
	
	# Initialize PortHandler Structs
	# Set the port path
	# Get methods and members of PortHandlerLinux or PortHandlerWindows
	
	# Open port
	if dynamixel.openPort(port_num):
	    print("Succeeded to open the port!")
	else:
	    print("Failed to open the port!")
	    print("Press any key to terminate...")
	    getch()
	    quit()

	# Set port baudrate
	if dynamixel.setBaudRate(port_num, BAUDRATE):
	    print("Succeeded to change the baudrate!")
	else:
	    print("Failed to change the baudrate!")
	    print("Press any key to terminate...")
	    getch()
	    quit()

	# Enable Dynamixel Torque
	for i in xrange(12):
		dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, i+1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
		    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
		elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
		    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
		else:
		    print("Dynamixel %d has been successfully connected" %(i+1))

	

def listener():

	rospy.init_node('listener2', anonymous=True)
	rospy.Subscriber('tester2', Data, callback)
	rospy.spin()

if __name__ == '__main__':
	dxl_init()
	listener()
