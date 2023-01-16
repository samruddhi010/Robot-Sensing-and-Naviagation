#!/usr/bin/python3

import rospy
import serial
from math import sin, pi
from std_msgs.msg import Float64, String
import utm
from custom_msg_python.msg import custom

if __name__ == '__main__':
    SENSOR_NAME = "gps_p"
    rospy.init_node('GPS_D')
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',4800)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
    
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    
    sampling_count = int(round(1/(sampling_rate*0.007913)))
    rospy.sleep(0.2)            
    rospy.logdebug("Initialization completed")    
    rospy.loginfo("Publishing GPS DATA.")
   
    pub = rospy.Publisher("GPSDATA", custom, queue_size= 10 )
    msg = custom()
   
   
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            
            if line == '':
                rospy.logwarn("No GPS data")
            else:
                if line.startswith(b'$GPGGA'):
                          
                
                    x=line.split(b",")
                    
                    
                    
                    lat = float(x[2])
                    
                    latdir = x[3]
                    
                    lon = float(x[4])
                    
                    londir = x[5]
                    
                    alti = float(x[9])
                    

                    degreeslat = int(lat) // 100
                    minuteslat = lat - 100*degreeslat
                    
                    latitude =degreeslat + minuteslat/60

                    degreeslong = int(lon) // 100
                    minuteslong = lon - 100*degreeslong
                   
                    longitude =degreeslong + minuteslong/60
                    

                    if latdir==b'S':
                        latitude=-latitude
                    

                    if londir==b'W':
                        longitude=-longitude
                    

                    Easting, Northing, Zone_Number, Zone_Letter= utm.from_latlon(latitude,longitude)
                    print(Easting, Northing, Zone_Number, Zone_Letter)
                    
                    msg.header.stamp = rospy.get_rostime()
                    msg.latitude = latitude
                    msg.longitude = longitude
                    msg.utm_easting = Easting
                    msg.utm_northing = Northing
                    msg.zone = Zone_Number
                    msg.letter = Zone_Letter
                    msg.altitude = alti
                    pub.publish(msg)
                
            
            

    except rospy.ROSInterruptException:
        port.close()