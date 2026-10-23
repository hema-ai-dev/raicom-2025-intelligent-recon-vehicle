#_*_coding:UTF-8_*_
#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
n=0

myposek=PoseStamped()
mypose=PoseStamped()
mypose.header.frame_id='map'
mypose.pose.position.x=1.8689
mypose.pose.position.y=0.136948
mypose.pose.position.z=0.0
mypose.pose.orientation.x=0
mypose.pose.orientation.y=0
mypose.pose.orientation.z=0.0143343
mypose.pose.orientation.w=0.999897


mypose1=PoseStamped()
mypose1.header.frame_id='map'
mypose1.pose.position.x=3.13407
mypose1.pose.position.y=0.203434
mypose1.pose.position.z=0
mypose1.pose.orientation.x=0
mypose1.pose.orientation.y=0
mypose1.pose.orientation.z=0.732289
mypose1.pose.orientation.w=0.680994

mypose2=PoseStamped()
mypose2.header.frame_id='map'
mypose2.pose.position.x=2.49585
mypose2.pose.position.y=0.767376
mypose2.pose.position.z=0
mypose2.pose.orientation.x=0
mypose2.pose.orientation.y=0
mypose2.pose.orientation.z=0.714842
mypose2.pose.orientation.w=0.699286

mypose3=PoseStamped()
mypose3.header.frame_id='map'
mypose3.pose.position.x=1.41488
mypose3.pose.position.y=0.629989
mypose3.pose.orientation.z=0.699643
mypose3.pose.orientation.w=0.714492

mypose4=PoseStamped()
mypose4.header.frame_id='map'
mypose4.pose.position.x=0.42563
mypose4.pose.position.y=0.586929
mypose4.pose.orientation.z=0.722881
mypose4.pose.orientation.w=0.690972

mypose5=PoseStamped()
mypose5.header.frame_id='map'
mypose5.pose.position.x=1.11061
mypose5.pose.position.y=2.06089
mypose5.pose.orientation.z=0.696204
mypose5.pose.orientation.w=0.717844

mypose6=PoseStamped()
mypose6.header.frame_id='map'
mypose6.pose.position.x=1.27455
mypose6.pose.position.y=2.891
mypose6.pose.orientation.z=0.336708
mypose6.pose.orientation.w=0.941609

mypose7=PoseStamped()
mypose7.header.frame_id='map'
mypose7.pose.position.x=1.89137
mypose7.pose.position.y=3.12397
mypose7.pose.orientation.z=-0.115646
mypose7.pose.orientation.w=0.993291

mypose8=PoseStamped()
mypose8.header.frame_id='map'
mypose8.pose.position.x=2.47596
mypose8.pose.position.y=2.21711
mypose8.pose.orientation.z=0.0144629
mypose8.pose.orientation.w=0.999895

mypose9=PoseStamped()
mypose9.header.frame_id='map'
mypose9.pose.position.x=3.07672
mypose9.pose.position.y=2.26885
mypose9.pose.orientation.z=0.707675
mypose9.pose.orientation.w=0.706538

mypose10=PoseStamped()
mypose10.header.frame_id='map'
mypose10.pose.position.x=3.08388
mypose10.pose.position.y=4.27432
mypose10.pose.orientation.z=0.999974
mypose10.pose.orientation.w=-0.00721063

mypose11=PoseStamped()
mypose11.header.frame_id='map'
mypose11.pose.position.x=2.4002
mypose11.pose.position.y=4.24262
mypose11.pose.orientation.z=-0.718852
mypose11.pose.orientation.w=0.695163

mypose12=PoseStamped()
mypose12.header.frame_id='map'
mypose12.pose.position.x=1.28874
mypose12.pose.position.y=4.24426
mypose12.pose.orientation.z=-0.684856
mypose12.pose.orientation.w=0.728679

mypose13=PoseStamped()
mypose13.header.frame_id='map'
mypose13.pose.position.x=-0.0243058
mypose13.pose.position.y=4.11642
mypose13.pose.orientation.z=-0.69458
mypose13.pose.orientation.w=0.719416

mypose14=PoseStamped()
mypose14.header.frame_id='map'
mypose14.pose.position.x=-0.134894
mypose14.pose.position.y=2.34742
mypose14.pose.orientation.z=-0.704298
mypose14.pose.orientation.w=0.709904

mypose15=PoseStamped()
mypose15.header.frame_id='map'
mypose15.pose.position.x=-0.0932979
mypose15.pose.position.y=-0.0157285
mypose15.pose.orientation.z=0.0213673
mypose15.pose.orientation.w=0.999772

def pose_callback(msg):
    global turtle_vel_pub
    global n
    if msg.status.status == 3:
        print("Y")
        n=n+1
        if n==1:
            turtle_vel_pub.publish(mypose1)
            time.sleep(1)
        if n==2:
            turtle_vel_pub.publish(mypose2)
            time.sleep(1)
        if n==3:
            turtle_vel_pub.publish(mypose3)
            time.sleep(1)
        if n==4:
            turtle_vel_pub.publish(mypose4)
            time.sleep(1)
        if n==5:
            turtle_vel_pub.publish(mypose5)
            time.sleep(1)
        if n==6:
            turtle_vel_pub.publish(mypose6)
            time.sleep(1)
        if n==7:
            turtle_vel_pub.publish(mypose7)
            time.sleep(1)
        if n==8:
            turtle_vel_pub.publish(mypose8)
            time.sleep(1)
        if n==9:
            turtle_vel_pub.publish(mypose9)
            time.sleep(1)
        if n==10:
            turtle_vel_pub.publish(mypose10)
            time.sleep(1)
        if n==11:
            turtle_vel_pub.publish(mypose11)
            time.sleep(1)
        if n==12:
            turtle_vel_pub.publish(mypose12)
            time.sleep(1)
        if n==13:
            turtle_vel_pub.publish(mypose13)
            time.sleep(1)
        if n==14:
            turtle_vel_pub.publish(mypose14)
            time.sleep(1)
        if n==15:
            turtle_vel_pub.publish(mypose15)
            time.sleep(1)


if __name__ == '__main__':
    rospy.init_node('pubpose')
    global turtle_vel_pub
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    turtle_vel_pub.publish(myposek)
    time.sleep(1)
    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, pose_callback)
    turtle_vel_pub.publish(mypose)
    time.sleep(1)
    rospy.spin()
