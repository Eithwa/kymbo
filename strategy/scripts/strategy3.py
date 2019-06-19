#!/usr/bin/env python
# -*- coding: utf-8 -*-+

# insert opencv 3.0 
import sys
#sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')
sys.path.insert(0,'/opt/ros/kinetic/lib/python2.7/dist-packages')
#sys.path.insert(1,'/usr/local/lib/lib/python2.7/dist-packages')


# ros lib
import rospy
import roslib
roslib.load_manifest('strategy')
import tf

import math
import numpy as np
import cv2

# rostopic msg
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from vision.msg import aim


# Define Behavior
INIT = 0
BEGINER = 1
FIND_BALL = 2
CATCH_BALL = 3
GOAL = 4
STRAIGHT = 5

# FLAG  
SIMULATEION_FLAG = False


def Rad2Deg(angle):
	return angle*180/math.pi

def Norm_Angle(angle):
	if(angle > 180):
		angle -=360
	elif(angle < -180):
		angle +=360
	return angle

class NodeHandle(object):
	def __init__(self):
		self._scan = None
		self._pos = None
		self._front = None

		self._start = 1

		self._ballsColor = []
		self._ballsDis = None
		self._ballsAng = None
		self._ballsArea = None
		
		if(SIMULATEION_FLAG):
			self.pub_vel = rospy.Publisher('robot1/cmd_vel',Twist, queue_size = 1)
		
			self.sub_scaninfo = rospy.Subscriber("robot1/scan",LaserScan,self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("robot1/odom",Odometry,self.Set_Odom)
		else:
			self.pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 1)
			
			self.sub_scaninfo = rospy.Subscriber("scan",LaserScan,self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("odom",Odometry,self.Set_Odom)
		
		self.sub_balls = rospy.Subscriber("tb3/ball",aim,self.Set_Balls)
	
		self.sub_start = rospy.Subscriber("strategy/start",Int32,self.Set_Start)

	def Set_ScanInfo(self,msg):
		self._scan = msg.ranges

	def Set_Odom(self,msg):
		self._pos = [msg.pose.pose.position.x,msg.pose.pose.position.y]
		(r,p,y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        
		self._front = Rad2Deg(y)
	def Set_Start(self,msg):
		self._start = msg.data

	def Set_Balls(self,msg):
		self._ballsColor = msg.color
		self._ballsDis = msg.dis
		self._ballsAng = msg.ang
		self._ballsArea = msg.area 		

class Strategy(NodeHandle):
	def __init__(self):
		super(Strategy,self).__init__()
		self.behavior = INIT

		#self.initpos = [[0.0,-1.95],[2.2,-1.95],[2.2,0.05]]
		#self.initpos = [[2.7,0.0]]
		#self.initpos = [[0.0,-0.3],[0.3,-0.3],[0.3,0.0]]
		#self.initpos = [[2.7,0.0],[1.5,0.0],[1.5,-0.35],[2.7,-0.35],[1.5,-0.35],[1.5,-0.66],[2.7,-0.66],[1.5,-0.66],[1.5,0.35],[2.7,0.35],[1.5,0.35],[1.5,0.66],[2.7,0.66]]
		self.initpos = [[2.7,0.0],[0.5,0.0],[0.5,-0.3],[2.7,-0.3],[0.5,-0.3],[0.5,-0.6],[2.7,-0.6],[0.5,-0.6],[0.5,0.3],[2.7,0.3],[0.5,0.3],[0.5,0.6],[2.7,0.6],[2.0,0.6]]

		self.findballpos = [2.2,0.05]
		#self.findballpos = [0.4,0.0]
		self.goal = None
		self._goalarea = {'red':[2.7,-0.6],'blue':[2.7,0.0],'yellow':[2.7,0.3],'white':[2.7,0.6],'black':[2.7,-0.3]}
		#self._goalarea = {'red':[0.5,-0.5],'blue':[0.5,0.0],'yellow':[0.5,0.25],'white':[0.5,0.5],'black':[0.5,-0.25]}
		self.prev_RPdis = 999

		self.state = 0
		
		# self.vel_x = 0.8
		self.vel_x = 0.3
		self.vel_z = 0.2
		self.error_ang = 1.0
		self.error_dis = 0.02
		self.error_area = 30000
		self.error_catchArea = 80000
		self.findang = 30
		self.error_ballang = 5.0

		self.ballcolor = None
		self.balldis = 999
		self.ballang = 999
		self.ballarea = 0
		
	def Process(self):
		if(self._start):
			print(self.state,self.behavior,self.goal,self.ballcolor,self.ballang)
			if(self.behavior == INIT):
				self.goal = self.initpos.pop(0)
				#self.behavior = BEGINiER
				self.behavior = STRAIGHT
				#print(self.goal)
			elif(self.behavior == BEGINER):
				self.Beginer_Strategy()
			elif(self.behavior == FIND_BALL):
				self.Find_Ball_Strategy()
			elif(self.behavior == CATCH_BALL):
				self.Catch_Ball_Strategy()
			elif(self.behavior == GOAL):
				self.Goal_Strategy()
			elif(self.behavior == STRAIGHT):
 				self.Straight_Strategy()
			else:
				self.Robot_Stop()		
		else:
			self.Robot_Stop()
			print('stop')

	def Robot_Vel(self,vel):
		twist = Twist()
		twist.linear.x = vel[0]
		twist.angular.z = vel[1]

		self.pub_vel.publish(twist)
	
	def Robot_Stop(self):
		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0
        
		self.pub_vel.publish(twist)   
	
    	# robot to pose angle
	def Get_RP_Angle(self,pos):
		return Rad2Deg(math.atan2((pos[1]-self._pos[1]),(pos[0]-self._pos[0])))
	
	# robot to pose distance
	def Get_RP_Dis(self,pos):
		return math.sqrt(pow(pos[0]-self._pos[0],2.0)+pow(pos[1]-self._pos[1],2.0))
	
	# strategy
	def Beginer_Strategy(self):

		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
						
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
			 		x = 0
					z = self.vel_z
				else:
					x = 0
					z = -self.vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) > self.error_ang):
						if(RPang > 0):
					 		x = self.vel_x-self.vel_z
							z = self.vel_z
						else:
							x = self.vel_x-self.vel_z
							z = -self.vel_z 
					else:
						x = self.vel_x
						z = 0				
				else:
					x = 0
					z = 0
					self.state = 0
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				if(len(self.initpos) == 0):
					self.behavior = FIND_BALL
				else:
					self.goal = self.initpos.pop(0)
					self.prev_RPdis = 999
				self.state = 0
				self.Robot_Stop()			
		else:
			self.Robot_Stop()

	def Find_Ball_Strategy(self):
		if(self.state == 0):
			if(self._front >= 0):
				RPang = Norm_Angle((90+self.findang)-self._front)
				# RPang = Norm_Angle((self.findang)-self._front)
			else:
				RPang = Norm_Angle(-(90+self.findang)-self._front)
				# RPang = Norm_Angle(-(self.findang)-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.vel_z
				else:
					x = 0
					z = -self.vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				if(self._front >= 0):
					self.state = 1
				else:
					self.state = 2
		elif(self.state == 1):
			RPang = Norm_Angle(-(90+self.findang)-self._front)
			# RPang = Norm_Angle(-(self.findang)-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.vel_z
				else:
					x = 0
					z = -self.vel_z
				self.Robot_Vel([x,z])
				for i in range(len(self._ballsColor)):
					if(self._ballsArea[i] > self.ballarea):
						self.ballcolor = self._ballsColor[i]
						self.ballarea = self._ballsArea[i]
						if(abs(self._ballsAng[i]) < self.error_ang):
							self.ballang = self._front
			else:
				self.Robot_Stop()
				self.behavior = CATCH_BALL
				self.state = 0
		elif(self.state == 2):
			RPang = Norm_Angle(90+self.findang-self._front)
			# RPang = Norm_Angle(self.findang-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.vel_z
				else:
					x = 0
					z = -self.vel_z
				self.Robot_Vel([x,z])
				for i in range(len(self._ballsColor)):
					if(self._ballsArea[i] >= self.ballarea):
						self.ballcolor = self._ballsColor[i]
						self.ballarea = self._ballsArea[i]
						self.ballang = self._front+self._ballsAng[i]
						# if(abs(self._ballsAng[i]) < self.error_ang):
    					# 		self.ballang = self._front
			else:
				self.Robot_Stop()
				self.behavior = CATCH_BALL
				self.state = 0
	def Catch_Ball_Strategy(self):
		if(self.state == 0):
			if(self.ballang == 999):
				self.Robot_Stop()
				self.behavior = FIND_BALL
				self.state = 0
				self.ballcolor = None
				self.ballarea = 0
				self.ballang = 999
			else:
				RBang = Norm_Angle(self.ballang-self._front)
				if(abs(RBang) > self.error_ang):
					if(RBang > 0):
						x = 0
						z = self.vel_z
					else:
						x = 0
						z = -self.vel_z
					self.Robot_Vel([x,z])
				else:
					self.Robot_Stop()
					self.state = 1
		elif(self.state == 1):
			color = None
			for i in range(len(self._ballsColor)):
				if(self._ballsColor[i] == self.ballcolor):
					color = self._ballsColor[i]
					break
			if(color):
				RBang = self._ballsAng[i]
				if(abs(RBang) > self.error_ang):
					if(RBang > self.error_ang):
						x = 0
						z = self.vel_z
					else:
						x = 0
						z = -self.vel_z
					self.Robot_Vel([x,z])
				else:
					self.Robot_Stop()
					self.state = 2
			else:
				self.Robot_Stop()
				self.state = 0
				self.behavior = FIND_BALL
				self.ballcolor = None
				self.ballang = 999
				self.ballarea = 0
		elif(self.state == 2):
			color = None
			for i in range(len(self._ballsColor)):
				if(self._ballsColor[i] == self.ballcolor):
					color = self._ballsColor[i]
					break
			if(color):
				print(color)
				RPdis = self._ballsArea[i]
				RBang = self._ballsAng[i]
				if(RPdis < self.error_area):
					if(self.prev_RPdis <= RPdis):
						if(abs(RBang) > self.error_ang):
							if(RBang > 0):
								x = self.vel_x-self.vel_z
								z = self.vel_z
							else:
								x = self.vel_x-self.vel_z
								z = -self.vel_z 
						else:
							x = self.vel_x-self.vel_z
							z = 0				
					else:
						x = 0
						z = 0
						self.state = 1
					self.Robot_Vel([x,z])
					self.prev_RPdis = RPdis
				else:
					self.behavior = GOAL
					self.goal = self._goalarea[color]
					self.state = 0
					self.Robot_Stop()
			else:
				self.state = 0
				self.behavior = FIND_BALL
				self.ballang = 999
				self.ballarea = 0
				self.ballcolor = None
				self.Robot_Stop()

	def Goal_Strategy(self):
		if(self.state == 0):
			color = None
			for i in range(len(self._ballsColor)):
				if(self._ballsColor[i] == self.ballcolor):
					color = self._ballsColor[i]
					break
			if(self.state == 0):
				RPang = Norm_Angle(self.Get_RP_Angle(self._goalarea[color])-self._front)
				#RBang = Norm_Angle(self._ballsAng[i])
				RBang = 0.0			
				if(abs(RPang) > self.error_ang):
					if(RPang > 0):
						if(abs(RBang) > self.error_ballang):
							#if(RBang > 0):
							#	x = 0
							#	z = self.vel_z
							#else:
							#	x = 0
							#	z = -self.vel_z
							x = 0
							z = self.vel_z
						else:
							x = 0
							z = self.vel_z
					else:
						if(abs(RBang) > self.error_ballang):
							if(RBang > 0):
								#x = 0
								#z = self.vel_z
								x = 0
								z = -self.vel_z
							else:
								x = 0
								z = -self.vel_z
						else:
							x = 0
							z = -self.vel_z
					self.Robot_Vel([x,z])
				else:
					self.Robot_Stop()
					self.state = 1
			else:
				self.Robot_Stop()
				self.state = 0
				self.behavior = FIND_BALL
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			# RBang = Norm_Angle(self._ballsAng[i])
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) > self.error_ang):
						if(RPang > 0):
					 		x = self.vel_x-self.vel_z
							z = self.vel_z
						else:
							x = self.vel_x-self.vel_z
							z = -self.vel_z 
					else:
						x = self.vel_x-self.vel_z
						z = 0				
				else:
					x = 0
					z = 0
					self.state = 0
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				x = -self.vel_x
				z = 0
				self.Robot_Vel([x,z])
				self.state = 2
		elif(self.state == 2):
			RPdis = self.Get_RP_Dis([self.findballpos[0],self.goal[1]])
			
			if(RPdis < 0.5):
				x = -self.vel_x
				z = 0
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				self.state = 0
				self.behavior = BEGINER
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
				self.ballarea = 0
				self.goal = self.findballpos

	def Straight_Strategy(self):
		ang = 0
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			ang = RPang			
			if(abs(RPang) > self.error_ang):
				if(abs(RPang) > 180 - 20):
					self.state = 2
					x = 0
					z = 0
					self.Robot_Stop() 
				elif(RPang > 0):
			 		x = 0
					z = self.vel_z
				else:
					x = 0
					z = -self.vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) > self.error_ang):
						if(RPang > 0):
					 		x = self.vel_x
							z = self.vel_z
						else:
							x = self.vel_x
							z = -self.vel_z 
					else:
						x = self.vel_x
						z = 0			
				else:
					x = 0
					z = 0
					self.state = 0
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				if(len(self.initpos) == 0):
					self.behavior = FIND_BALL
				else:
					self.goal = self.initpos.pop(0)
					self.prev_RPdis = 999
				self.state = 0
				self.Robot_Stop()
		elif(self.state == 2):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			if(RPdis > self.error_dis):
					
				#x = -self.vel_x+self.vel_z
				#z = 0
				#self.Robot_Vel([x,z])
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) < 180-self.error_ang):
						if(RPang > 0 and RPang < 180-self.error_ang):
					 		x = -self.vel_x
							z = -self.vel_z
						else:
							x = -self.vel_x
							z = self.vel_z 
					else:
						x = -self.vel_x
						z = 0			
				else:
					x = 0
					z = 0
					self.state = 0
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				if(len(self.initpos) == 0):
					self.behavior = FIND_BALL
				else:
					self.goal = self.initpos.pop(0)
					self.prev_RPdis = 999
				self.state = 0
				self.Robot_Stop()			
		else:
			self.Robot_Stop()		
			


def main():
	rospy.init_node('strategy', anonymous=True)
	strategy = Strategy()

	# 30 hz
	rate = rospy.Rate(30)
	i = 0
	while not rospy.is_shutdown():
	#if(strategy._scan):
	#for i in range(len(strategy._scan)):
	#    print(str(i)+"  "+str(strategy._scan[i]))
	#print("0 : " + str(strategy._scan[0]))
	#print("90 : " + str(strategy._scan[90]))
	#print("180 : " + str(strategy._scan[180]))
	#print("270 : " + str(strategy._scan[270]))
		if(strategy._pos):        
			strategy.Process()
		rate.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == "__main__":
	main()
