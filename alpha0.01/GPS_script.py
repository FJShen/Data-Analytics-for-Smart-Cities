import rospy
import serial
import re
from math import pi
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def trial():
    with serial.Serial('/dev/ttyTHS2', 9600, timeout=5) as ser:
        junk = ser.readline()
	junk = ser.readline()
	line = ser.readline()
	junk = ser.readline()
	junk = ser.readline()
	junk = ser.readline()
	junk = ser.readline()
	junk = ser.readline()
	print(line) 
    
    #testing
    #line = "$GNGGA,182011.00,4026.04852,N,08654.64548,W,1,09,1.03,188.6,M,-33.4,M,,*71"
    
    data = []
    gps_coordinates_match = re.search("\$GNGGA,([\d]*\.?[\d]{,2}),([\d]*\.?[\d]{,5}),([A-Z]?),([\d]*\.?[\d]{,5}),([A-Z]?)", line)

    if gps_coordinates_match is not None:
        data = list(gps_coordinates_match.groups())

    if len(data) < 5:
	print(len(data))
	raise ValueError('Invalid data')
		
    return data

def _format_data(data):
    if data[0] != "":
       data[0] = int(float(data[0]))
    else:
       data[0] = 0

    if data[1] != "":
       int_part = data[1][:2]
       int_part = int(int_part)#40
       dec_part = data[1][2:]
       dec_part = float(dec_part)#.2579051
       dec_part = dec_part/60#.4289183
       data[1] = round(float(int_part + dec_part), 7)
    else:
       data[1] = 0

    if data[3] != "":
       int_part = data[3][:3]
       int_part = int(int_part)#086
       dec_part = data[3][3:]
       dec_part = float(dec_part)#.5462594
       dec_part = dec_part/60#.0.9104323
       data[3] = round(float(int_part + dec_part), 7)

    else:
       data[3] = 0

    print("After:", data)
	
    return data

def test(string):
    pub_data = rospy.Publisher('gps_meas', String, queue_size=1)
    rate = rospy.Rate(1) # 1hz

    seq = 0

    while not rospy.is_shutdown():
        try:
       	     string = "Test"

        except ValueError as e:
	     print("ValueError")
             continue
 
    	pub_data.publish(string)
    	rate.sleep()
    	seq += 1

def talker(gps_meas):
    pub_data = rospy.Publisher('gps_meas', Odometry, queue_size=1)
    rate = rospy.Rate(1) # 1hz

    seq = 0

    while not rospy.is_shutdown():
        try:
       	     data = trial()
	     data = _format_data(data)

        except ValueError as e:
	     print("ValueError")
             continue

    	gps_meas.header.stamp = rospy.Time.now()                      #time of gps measurement
    	gps_meas.header.frame_id = "base_footprint"          		 #the tracked robot frame
    	gps_meas.pose.pose.position.x = data[3]              #x measurement GPS.
    	gps_meas.pose.pose.position.y = data[1]              #y measurement GPS.
    	gps_meas.pose.pose.position.z = 0              	 #z measurement GPS.
    	gps_meas.pose.pose.orientation.x = 1               	 #identity quaternion
    	gps_meas.pose.pose.orientation.y = 0               	 #identity quaternion
    	gps_meas.pose.pose.orientation.z = 0               	 #identity quaternion
    	gps_meas.pose.pose.orientation.w = 0               	 #identity quaternion
    	gps_meas.pose.covariance = [-1, 0, 0, 0, 0, 0,  	 #covariance on gps_x
    	                            0, -1, 0, 0, 0, 0,  	 #covariance on gps_y
    	                            0, 0, -1, 0, 0, 0,  	 #covariance on gps_z
    	                            0, 0, 0, 99999, 0, 0,  	 #large covariance on rot x
    	                            0, 0, 0, 0, 99999, 0,  	 #large covariance on rot y
    	                            0, 0, 0, 0, 0, 99999]  	 #large covariance on rot z
 
    	pub_data.publish(gps_meas)
    	rate.sleep()
    	seq += 1
   

gps_meas = Odometry()
string = String()

if __name__ == '__main__':
    rospy.init_node("gps_node")

    try:
        talker(gps_meas)
	# test(string) # DEBUG
    except rospy.ROSInterruptException:
        pass
