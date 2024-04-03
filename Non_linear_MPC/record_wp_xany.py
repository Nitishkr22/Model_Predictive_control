from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA
from novatel_oem7_msgs.msg import BESTUTM
# from novatel_gps_msgs.msg import 
from nav_msgs.msg import Odometry

import rospy
import time

x_pos = 0.0
y_pos = 0.0
gnss_vel = 0.0
# def callback_latlong(data):
#     global lat,lng
#     lat = data.lat
#     lng = data.lon

#GNSS heading
# def callback_heading(data):
#     global heading
#     heading=data.azimuth

def callback_xandy(data):
    global easting, northing
    easting = data.pose.pose.position.x
    northing = data.pose.pose.position.y

def callback_vel(data):
    global gnss_vel
    gnss_vel = data.hor_speed

def callback_heading(data):
    global north_vel, east_vel,heading
    north_vel = data.north_velocity
    east_vel = data.east_velocity
    heading = data.azimuth
      

rospy.init_node('Navigation', anonymous=True)
#ROS subscription
# rospy.Subscriber("/novatel/oem7/bestpos",BESTPOS, callback_latlong)
rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)
rospy.Subscriber("/novatel/oem7/bestvel",BESTVEL, callback_vel)
rospy.Subscriber("/novatel/oem7/odom",Odometry, callback_xandy)



global new_file
new_file=str(int(time.time()))+".txt"
file=open(new_file,"w")

while not rospy.is_shutdown():
    # global lat,lng
    time.sleep(0.1)
    file = open(str(new_file), "a")
    print("done")
    file.writelines("["+str(time.time())+","+str(easting)+","+str(northing)+","+str(north_vel)+","+str(east_vel)+","+str(heading)+"],\n")
    file.close()
    # print(x_pos,y_pos)
