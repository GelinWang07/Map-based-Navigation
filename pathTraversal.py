import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from math import atan2, pi
import actionlib
import tf2_ros
import tf
from gridGeneration import myPoint
from gridGeneration import *
from geometry_msgs.msg import Point, Twist, Quaternion,PointStamped
from startGridExploration import endGoalX, endGoalY, generated_points_red,unreachedGoalX,unreachedGoalY

def getRobotPosition(data):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('map', 'base_link' ,rospy.Time(0), rospy.Duration(10.0))
        print("Translation (X,Y,Z): (" + str(trans.transform.translation.x) + ", " + str(trans.transform.translation.y) + ", " + str(trans.transform.translation.z) + ")")
        print("Rotation (X,Y,Z,W): (" + str(trans.transform.rotation.x) + ", " + str(trans.transform.rotation.y) + ", " + str(trans.transform.rotation.z) + ", " + str(trans.transform.rotation.w) + ")")
        cell_x = ((trans.transform.translation.x - data.info.origin.position.x) / data.info.resolution)
        cell_y = ((trans.transform.translation.y - data.info.origin.position.y) / data.info.resolution)
        cell_x = int(round(cell_x))
        cell_y = int(round(cell_y))
        print("cell_x:" + str(cell_x))
        print("cell_y:" + str(cell_y))
        return myPoint(cell_x, cell_y)

    except tf2_ros.TransformException as ex:
        print("error")
        print(type(ex))


def moveToGoal(bestGoal, data,lastgoal):
    global endGoalX
    global endGoalY
    global unreachedGoalX
    global unreachedGoalY
    global generated_points_red
    print(lastgoal)
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("waiting for the move_base action server to come up")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()

    msgGoalX = (bestGoal.x * data.info.resolution)  + data.info.origin.position.x
    msgGoalY = (bestGoal.y * data.info.resolution)  + data.info.origin.position.y

    currentPos = getRobotPosition(data)
    cell_currentPosX = currentPos.x
    cell_currentPosY = currentPos.y
    currentX = (cell_currentPosX * data.info.resolution)  + data.info.origin.position.x
    currentY = (cell_currentPosY * data.info.resolution)  + data.info.origin.position.y

    goal.target_pose.pose.position = Point(msgGoalX,msgGoalY,0)
    goal.target_pose.pose.orientation =  getGoalAngleQuat(currentX,currentY,msgGoalX,msgGoalY)

    rospy.loginfo("Sending goal location...")
    print(goal.target_pose.pose.position)
    print(goal.target_pose.pose.orientation)
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("the destination is reached")
        endGoalX.append(msgGoalX)
        endGoalY.append(msgGoalY)
        #print(np.arange(generated_points_red-1,len(endGoalX) + generated_points_red-1))
        drawGoalsInRviz(endGoalX,endGoalY,[1.0,0.0,0.0],np.arange(generated_points_red,len(endGoalX) + generated_points_red))
        if lastgoal:
            generated_points_red += len(endGoalX)
            endGoalX = []
            endGoalY = []
        return True
    else:
        rospy.loginfo("failed to reach destination")
        unreachedGoalX.append(msgGoalX)
        unreachedGoalY.append(msgGoalY)
        drawGoalsInRviz(endGoalX,endGoalY,[1.0,0.0,1.0],np.arange(generated_points_red,len(endGoalX) + generated_points_red))        

        return False



def rotate(speed, angle, clockwise):
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    # speed(degrees/sec)
    angular_speed = speed*pi/180 
    # distance(degrees)
    relative_angle = angle*pi/180

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        vel_pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #force to stop
    print("a whole rotate finished. Stop now")
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)

def getGoalAngleQuat(currentPosX,currentPosY,goalX,goalY):
    steering_angle = atan2(goalY-currentPosY,goalX-currentPosX)
    quat = tf.transformations.quaternion_from_euler(0,0,steering_angle)
    msgQuat = Quaternion(*quat)
    print("quat is:" + str(msgQuat))
    return msgQuat
