#!/usr/bin/env python
import rospy
import math, random
import os

# ARGOS MSGS
from argos_ros_epuck_foraging.msg import Proximity
from argos_ros_epuck_foraging.msg import ProximityList
from argos_ros_epuck_foraging.msg import Neighbor
from argos_ros_epuck_foraging.msg import NeighborList
from argos_ros_epuck_foraging.msg import Leaf
from argos_ros_epuck_foraging.msg import MerkleLeafList

# ROS MSGS
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from diff_drive.msg import *
from std_msgs.msg import Bool, Float32, String, ColorRGBA
import message_filters

# CUSTOM CLASSES
from merkle_tree import SwarmMerkleTree
from auxiliary_functions import *

class Controller:

    # Publishers Handlers
    cmdVelPub = None
    cmdRGBLedsPub = None
    cmdGoToPub = None
    merkleCompletedPub = None

    # Navigation Variables
    pose = None
    lastTwist = None

    # Time Variables
    time = 0
    stateStartTime = 0

    # State Variables
    state = "WANDER"
    CurrentRGBColor = None
    previous_state = True
    goalAchieved = False

    # Puck-related variables
    alignmentThreshold = 0.02
    grabbingDistancePuckThreshold = 16

    # Constants
    MAX_FORWARD_SPEED = 0.15
    MAX_ROTATION_SPEED = 0.30

    # Goal Pose
    goal = Pose()
    goal.position.x = 1.25
    goal.position.y = 1.25

    # Cell Variables
    CurrentCell = Pose()
    cell_index = -1
    dimensionOfCell = 0.5

    # Random Pose
    random_pose = Pose()
    random_pose.position.x =  round(random.uniform(1,2), 2)
    random_pose.position.y =  round(random.uniform(1,2), 2)

    # Current Goal
    current_goal_info = None
    current_goal_status = None
    current_goal_status_id = None

    # Internal Merkle Tree
    mk = None
    merkle_leafs = 0

    def __init__(self):

        # Publishers
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmdRGBLedsPub = rospy.Publisher('rgb_leds', String, queue_size=1)
        self.cmdGoToPub = rospy.Publisher('diff_drive_go_to_goal/go_to', PoseStamped, queue_size=1)
        self.cmdCancelGoalPub = rospy.Publisher('diff_drive_go_to_goal/cancel', GoalID, queue_size=1)
        self.merkleCompletedPub = rospy.Publisher('comm/merkle_completed', MerkleLeafList, queue_size=1)

        # Subscribers
        rospy.Subscriber('position', Pose, self.pose_callback)
        rospy.Subscriber('color_list', ColorRGBA, self.color_callback)
        rospy.Subscriber('proximity', ProximityList, self.prox_callback)
        rospy.Subscriber('diff_drive_go_to_goal/goal', GoToPoseActionGoal, self.current_goal_callback)
        rospy.Subscriber('diff_drive_go_to_goal/status', GoalStatusArray, self.current_goal_status_callback)
        rospy.Subscriber('diff_drive_go_to_goal/goal_achieved', Bool, self.goal_achieved_callback)
        rospy.Subscriber('comm/neighbor_list', NeighborList, self.neighbors_callback)

        # Received Merkle Tree
        received_merkle_tree = rospy.client.wait_for_message(rospy.get_namespace() + "comm/init_merkle_tree", String, timeout=None)
        self.mk = SwarmMerkleTree(received_merkle_tree.data)

        # Navigation variables
        self.lastTwist = self.twistRandom()

    def neighbors_callback(self, NeighborList):
        if(NeighborList):
            for neighbor in NeighborList.neighbors:
                neighbor_topic_name = '/epuck_' + str(neighbor.ID) + '/comm/merkle_completed'
                try:
                    neighbor_merkle =  rospy.client.wait_for_message(neighbor_topic_name, MerkleLeafList, timeout=0.5)
                    for leaf_index in xrange(neighbor_merkle.n):
                        if(neighbor_merkle.leafs[leaf_index].completed != self.mk.completed_merkle[leaf_index]['completed']):
                            if(neighbor_merkle.leafs[leaf_index].completed == True):
                                if(self.mk.check(neighbor_merkle.leafs[leaf_index].hash_action, neighbor_merkle.leafs[leaf_index].hash_input, leaf_index)):
                                    self.mk.completed_merkle[leaf_index]['completed'] = neighbor_merkle.leafs[leaf_index].completed
                                    self.mk.completed_merkle[leaf_index]['hash_action'] = neighbor_merkle.leafs[leaf_index].hash_action
                                    self.mk.completed_merkle[leaf_index]['hash_input'] = neighbor_merkle.leafs[leaf_index].hash_input
                                    self.mk.completed_merkle[leaf_index]['received'] = True
                                    self.mk.completed_merkle[leaf_index]['from_robot'] = neighbor.ID

                except rospy.ROSException as exc:
                    pass

    def current_goal_callback(self, current_goal):
        self.current_goal_info = current_goal

    def current_goal_status_callback(self, current_goal_status):
        self.current_goal_status = current_goal_status
        if (not self.current_goal_status):
            self.current_goal_status_id = -1
        else:
            if (not self.current_goal_status.status_list):
                self.current_goal_status_id = -1
            else:
                self.current_goal_status_id = self.current_goal_status.status_list[0].status

    def goal_achieved_callback(self, goalAchieved):
        self.goalAchieved = goalAchieved.data

    def pose_callback(self, pose):
        self.pose = pose

    def prox_callback(self, proxList):
        self.time += 1
        # Find the closest obstacle (other robot or wall).
        # The closest obstacle is the one with the greatest 'value'.
        closestObs = None
        highestValue = 0.05
        self.proxList = proxList

        for prox in proxList.proximities:
            if prox.value > highestValue:
                closestObs = prox
                highestValue = prox.value

        self.state_transitions(closestObs,closestPuck)
        self.state_actions(closestObs,closestPuck)

    # State transitions
    def state_transitions(self, closestObs, currentCellColor):
        if self.state == "AVOID":
            # Only leave this state upon timeout.
            if self.time - self.stateStartTime > random.randint(10,25):
                self.transition("WANDER")

        elif self.state == "HANDLE":
            if closestObs != None and self.CurrentRGBColor == None:
                self.transition("AVOID")
            elif self.goalAchieved:
                self.transition("LEAVE_PUCK")
            else:
                pass

        elif self.state == "WANDER":
            if closestObs != None:
                self.transition("AVOID")
            elif currentCellColor != None:
                if(self.checkColor(currentCellColor)):
                    if(not self.isColorAtTarget(currentCellColor)):
                        self.transition("HANDLE")
            else:
                pass

        elif self.state == "LEAVE_PUCK":
            # Only leave this state upon timeout.
            if self.time - self.stateStartTime > 50:
                self.transition("WANDER")
        else:
            die("Error: Invalid state")

    # State actions
    def state_actions(self,closestObs,currentCellColor):
        twist = None

        if self.state == "AVOID":
            if closestObs == None:
                twist = self.turnRobot()
            else:
                twist = self.twistTowardsThing(closestObs, True)

        elif self.state == "HANDLE":
            if (self.CurrentRGBColor == None):
                if currentCellColor != None:
                    if (self.checkColor(currentCellColor)):
                        if currentCellColor.range >= self.grabbingDistancePuckThreshold:
                            twist = self.twistTowardsThing(closestPuck)
                        else:
                            twist = self.alignWithColor(currentCellColor,self.alignmentThreshold)
                            if math.fabs(currentCellColor.angle) < self.alignmentThreshold:
                                self.grabPuck(currentCellColor)
                                self.goToGoal()
                    else:
                        self.transition("WANDER")
                else:
                    self.transition("WANDER")

        elif self.state == "LEAVE_PUCK":
            self.markColorAsCompleted()
            self.goalAchieved = False
            self.releaseColor()
            twist = self.twistTowardsThing(closestPuck,True)

        elif self.state == "WANDER":
            if(self.isRobotAtTarget()):
                if(currentCellColor != None):
                    twist = self.twistAway()
                else:
                    twist = self.turnRobot()
            else:
                twist = self.twistRandom()
        else:
            die("Error: Invalid state")

        self.publishCompletedMerkleTree()
        self.cmdVelPub.publish(twist)
        self.lastTwist = twist

    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.time

    def twistTowardsThing(self, thing, backwards=False):
        v = 0
        w = 0

        if math.fabs(thing.angle) < 0.5:
            # The thing is roughly forwards, go towards it
            if backwards:
                v = -self.MAX_FORWARD_SPEED
            else:
                v = self.MAX_FORWARD_SPEED
        elif thing.angle < 0:
            # Turn right
            w = -self.MAX_ROTATION_SPEED
        elif thing.angle > 0:
            # Turn left
            w = self.MAX_ROTATION_SPEED

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistAway(self):
        twist = Twist()
        twist.linear.x = -self.MAX_FORWARD_SPEED
        twist.angular.z = self.MAX_ROTATION_SPEED
        return twist

    def twistRandom(self):
        twist = Twist()
        twist.linear.x = self.MAX_FORWARD_SPEED * random.random()
        twist.angular.z = self.MAX_ROTATION_SPEED * (random.random() - 0.5)
        return twist

    def turnRobot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = self.MAX_ROTATION_SPEED
        return twist

    def turnRobotRandomDirection(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = self.MAX_ROTATION_SPEED * ((random.random() - 0.5)*2)
        return twist

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        return twist

    def goToGoal(self):
        self.goToPosition(self.goal)

    def goToRandomPosition(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.z = 0
        goal.pose.position.x = self.random_pose.position.x
        goal.pose.position.y = self.random_pose.position.y
        goal.pose.orientation.w = 0
        self.cmdGoToPub.publish(goal)

    def goToPosition(self, pose):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.z = pose.position.z
        goal.pose.position.x = pose.position.x
        goal.pose.position.y = pose.position.y
        goal.pose.orientation.w = pose.orientation.w
        self.cmdGoToPub.publish(goal)

    def CancelGoal(self):
        goal = GoalID()
        goal.id = self.current_goal_info.goal_id.id
        goal.stamp = rospy.Time(0,0)
        self.cmdCancelGoalPub.publish(goal)
        self.previous_state = True

    def grabColor(self, color):
        self.CurrentRGBColor = GetPuckColorBasedOnType(color)
        self.cmdRGBLedsPub.publish(self.CurrentRGBColor)

    def releaseColor(self):
        self.CurrentRGBColor = None
        self.cmdRGBLedsPub.publish("off")

    def checkColor(self, color):
        for index in xrange(self.mk.get_number_of_leafs()):
            self.leaf_index = index
            sensor_input = color
            action = 'HANDLE'
            if(self.mk.check(self.mk.generate_hash(action), self.mk.generate_hash(sensor_input), self.leaf_index)):
                return True

    def publishCompletedMerkleTree(self):
        CompletedMerkleTree = MerkleLeafList()
        CompletedMerkleTree.header.stamp = rospy.Time.now()
        CompletedMerkleTree.n = len(self.mk.completed_merkle)

        for leaf_index in xrange(CompletedMerkleTree.n):
            leaf = Leaf()
            leaf.ID = self.mk.completed_merkle[leaf_index]['id']
            leaf.completed = self.mk.completed_merkle[leaf_index]['completed']
            leaf.hash_action = self.mk.completed_merkle[leaf_index]['hash_action']
            leaf.hash_input = self.mk.completed_merkle[leaf_index]['hash_input']
            leaf.received = self.mk.completed_merkle[leaf_index]['received']
            leaf.from_robot = self.mk.completed_merkle[leaf_index]['from_robot']
            CompletedMerkleTree.leafs.append(leaf)

        self.merkleCompletedPub.publish(CompletedMerkleTree)

    def markColorAsCompleted(self):
        color_index = GetPuckTypeBasedOnColor(self.CurrentRGBColor) - 1
        self.mk.complete_leaf(color_index)

    def isMerkleTreeCompleted(self):
        return self.mk.is_complete_merkle_done()

    def isRobotAtTarget(self):
        if (self.GroundSensorPack.Left.value == 0 and\
            self.GroundSensorPack.Center.value == 0 and\
            self.GroundSensorPack.Right.value == 0):
            return True
        else:
            return False

    def isPuckAtTarget(self, Puck):
        X,Y,Z = quaternion_to_euler_angle(self.pose)
        theta = math.radians(Z) + Puck.angle
        distanceToPuck = Puck.range/100
        threshold = 0.475
        puckx = self.pose.position.x + (distanceToPuck*math.cos(theta))
        pucky = self.pose.position.y + (distanceToPuck*math.sin(theta))

        if (puckx >= (self.goal.position.x - threshold) and\
            puckx <= (self.goal.position.x + threshold) and\
            pucky >= (self.goal.position.y - threshold) and\
            pucky <= (self.goal.position.y + threshold)):
            return True
        else:
            return False

if __name__ == '__main__':
    rospy.init_node("controller")
    controller = Controller()
    rospy.spin()
