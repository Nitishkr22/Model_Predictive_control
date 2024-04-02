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
    global x_pos, y_pos
    x_pos = data.pose.pose.position.x
    y_pos = data.pose.pose.position.y

def callback_vel(data):
    global gnss_vel
    gnss_vel = data.hor_speed
      

rospy.init_node('Navigation', anonymous=True)
#ROS subscription
# rospy.Subscriber("/novatel/oem7/bestpos",BESTPOS, callback_latlong)
# rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)
rospy.Subscriber("/novatel/oem7/bestvel",BESTVEL, callback_vel)
rospy.Subscriber("/novatel/oem7/odom",Odometry, callback_xandy)



global new_file
new_file=str(int(time.time()))+".txt"
file=open(new_file,"w")

while not rospy.is_shutdown():
    # global lat,lng
    time.sleep(0.2)
    file = open(str(new_file), "a")
    print("done")
    file.writelines("["+str(time.time())+","+str(x_pos)+","+str(y_pos)+","+str(gnss_vel)+"],\n")
    file.close()
    # print(x_pos,y_pos)
