#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TransformStamped, PointStamped, PoseWithCovarianceStamped, PoseStamped
#from turtlesim.msg import Pose
from actionlib_msgs.msg import *
from robotnik_msgs.msg import Pose
from robotnik_msgs.srv import drive, driveResponse, path_follow_service
from math import pow, atan2, sqrt, cos, sin, pi
import tf_conversions
from std_srvs.srv import *
import time
from __main__ import *

ROBOT_ID = 'rb1_base_b' #this should be taken from the ROS param server
 
class TurtleBot:
 
	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node('drive_with_orientation', anonymous=True)

		#self.goal_subscriber = rospy.Subscriber('vicon/klt02/klt02',
		#                                 TransformStamped, self.update_goal_pose)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel',
					           Twist, queue_size=10)

		# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
		# when a message of type Pose is received.
		self.pose_subscriber = rospy.Subscriber('vicon/'+ROBOT_ID+'/'+ROBOT_ID+'',
					         TransformStamped, self.update_pose)

		self.collision_subscriber = rospy.Subscriber('/'+ROBOT_ID+'/move_safety_controller/warning_collision_point',
					         PointStamped, self.update_collision)
		self.goal = TransformStamped()
		self.collision = PointStamped()
		self.pose = TransformStamped()
		self.path_planning_goal = Pose()
		self.rate = rospy.Rate(10)
		self.vel_msg = Twist()
		self.error_theta= 1.0
		self.status= ''
		self.time_limit = 3
		self.path_feedback = 'failure'
		self.goal.header.seq = 0

	def update_collision(self, data):
		self.collision = data
		#print(data)	

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is received by the subscriber."""
		#print('update pose')
		self.pose = data
		self.pose.transform.translation.x = round(self.pose.transform.translation.x, 6)
		self.pose.transform.translation.y = round(self.pose.transform.translation.y, 6)

	def goal_angle(self):
		current_rot= [self.pose.transform.rotation.x, self.pose.transform.rotation.y, self.pose.transform.rotation.z, self.pose.transform.rotation.w]
		current_theta= tf_conversions.transformations.euler_from_quaternion(current_rot)
		final_rot= [self.goal.transform.rotation.x, self.goal.transform.rotation.y, self.goal.transform.rotation.z, self.goal.transform.rotation.w]
		final_theta= tf_conversions.transformations.euler_from_quaternion(final_rot)
		final_theta = final_theta[2] - current_theta[2]
		final_theta = atan2(sin(final_theta),cos(final_theta))
		return final_theta

	def goal_angular_vel(self, constant=0.8):
		return constant * (self.goal_angle())

	def goal_angle_error(self):
		current_rot= [self.pose.transform.rotation.x, self.pose.transform.rotation.y, self.pose.transform.rotation.z, self.pose.transform.rotation.w]
		current_theta= tf_conversions.transformations.euler_from_quaternion(current_rot)
		final_rot= [self.goal.transform.rotation.x, self.goal.transform.rotation.y, self.goal.transform.rotation.z, self.goal.transform.rotation.w]
		final_theta= tf_conversions.transformations.euler_from_quaternion(final_rot)
		theta_error = final_theta[2] - current_theta[2]
		return theta_error

	def update_goal_pose(self, data):
		"""Callback function which is called when a new message of type Pose is received by the subscriber."""
		#print('update pose')
		self.goal = data
		self.goal.transform.translation.x = round(self.goal.transform.translation.x, 6)
		self.goal.transform.translation.y = round(self.goal.transform.translation.y, 6)

		#the new goal for path planning is 10 cm away from the actual goal
		self.path_planning_goal.translation.x = self.goal.transform.translation.x
		self.path_planning_goal.translation.y = self.goal.transform.translation.y
		#return

	def euclidean_distance(self):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((self.goal.transform.translation.x - self.pose.transform.translation.x), 2) +
		     pow((self.goal.transform.translation.y - self.pose.transform.translation.y), 2))

	def linear_vel(self, constant=0.8):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * self.euclidean_distance()

	def steering_angle(self):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return atan2(self.goal.transform.translation.y - self.pose.transform.translation.y, self.goal.transform.translation.x - self.pose.transform.translation.x)

	def angular_vel(self, constant=0.8):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		rot=[self.pose.transform.rotation.x, self.pose.transform.rotation.y, self.pose.transform.rotation.z, self.pose.transform.rotation.w]
		your_euler = tf_conversions.transformations.euler_from_quaternion(rot)
		theta = your_euler[2]
		global error_theta
		self.error_theta= self.steering_angle() - theta
		self.error_theta= atan2(sin(self.error_theta),cos(self.error_theta))
		#print(error_theta)
		return constant * self.error_theta

	def velocity_status(self, data):
		self.vel_status = Twist()
		self.vel_status = data

	def simple_goal_status(self, data):
		self.sim_goal_status = GoalStatusArray()
		self.sim_goal_status = data

	def call_path_follow_server(self):
		print('Attempting to reach goal using navigation stack')
		move_simple_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base_simple/goal', PoseStamped, queue_size=10)
		move_simple_msg = PoseStamped()
		move_simple_msg.header.frame_id = 'vicon_world'
		move_simple_msg.pose.position.x = self.path_planning_goal.translation.x
		move_simple_msg.pose.position.y = self.path_planning_goal.translation.y
		move_simple_msg.pose.orientation.z = self.goal.transform.rotation.z
		move_simple_msg.pose.orientation.w = self.goal.transform.rotation.w
		rospy.sleep(0.3)
		move_simple_pub.publish(move_simple_msg)
		goal_status = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.simple_goal_status)
		vel_status = rospy.Subscriber('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, self.velocity_status)
		rospy.sleep(0.1)
		timer = time.time()
		while True:
			if(len(self.sim_goal_status.status_list)==1):
				if(self.sim_goal_status.status_list[0].text == 'Goal reached.'):
					print('SIMPLE Goal reached')
					self.move2goal()
					break
				elif(self.sim_goal_status.status_list[0].text == 'Failed to find a valid plan. Even after executing recovery behaviors.'):
					print('Navigation Stack Failed - Requesting UNITY path')
					rospy.wait_for_service('path_follow_server')
					service_object = rospy.ServiceProxy('path_follow_server', path_follow_service)
					ros_response = service_object(self.path_planning_goal, ROBOT_ID)
					print ros_response.feedback
					self.path_feedback = ros_response.feedback
					if((self.path_feedback == 'Finished') or ((self.path_feedback == 'Please Open Unity OS'))):
						print('UNITY', self.path_feedback)
						#if(self.path_feedback == 'Please Open Unity OS'):
							#print('Failed to obtain UNITY path - Moving to goal with collision avoidance')
						self.move2goal()
						break
				#elif((self.vel_status.linear.x <= 0.05) and ((time.time()-timer) >= 3.0)):
				#	rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps')
				#	reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
				else:
					pass
		print('Exit Path Following loop')
		#rospy.wait_for_service('path_follow_server')
		#service_object = rospy.ServiceProxy('path_follow_server', path_follow_service)
		#ros_response = service_object(self.path_planning_goal, ROBOT_ID)
		#print ros_response.feedback
		#self.path_feedback = ros_response.feedback
		#if((self.path_feedback == 'Finished') or ((self.path_feedback == 'Please Open Unity OS'))):
			#self.move2goal()
		return
	 
	def move2goal(self):
		"""Moves the turtle to the goal."""
		rospy.sleep(0.2)
		#print self.goal
		# Please, insert a number slightly greater than 0 (e.g. 0.01).
		# distance_tolerance = input("Set your tolerance: ")
		distance_tolerance = 0.05
		heading_tolerance = 0.5
		orientation_tolerance = 0.01
		vel_msg = Twist()
		collision_tolerence = PointStamped()
		collision_tolerence.point.x = 0.3
		collision_tolerence.point.y = 0.30
		self.collision.point.x= 1.0
		self.collision.point.y= 1.0
		last_seq_collision= 0
		last_seq_vicon= 0
		last_collision_point = 0
		collision_time_limit =10
		collision_timer = time.time()
		while ((self.euclidean_distance() >= distance_tolerance)):
			while((abs(self.error_theta) > heading_tolerance)):

				# Linear velocity in the x-axis.
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = self.angular_vel()

				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)

######################################################### going to the goalllll

			vel_msg.linear.x = self.linear_vel()
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

			# Angular velocity in the z-axis.
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel()
			# Publishing our vel_msg
			self.velocity_publisher.publish(vel_msg)
			#print 'comes after veloctiy'

######################################################### Obstacle avoidance

			if(((self.collision.point.x <= collision_tolerence.point.x) or (abs(self.collision.point.y) <= collision_tolerence.point.y)) and (self.collision.header.seq > last_seq_collision) and (self.euclidean_distance() > 0.7)):
	
				print('Collision is detected')
				self.status= 'Collision is detected'
				print('Entered collision avoidance procedure')
				last_seq_collision = self.collision.header.seq
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = 0
				self.velocity_publisher.publish(self.vel_msg)
				#last_collision_point = self.collision.point.x
				print('rotating')
				self.rotate_in_place()
				print('done rotating')
				print('driving')
				self.drive()
				print('done driving')
				print('Exited collision avoidance procedure')
				#rospy.sleep(0.2)
				self.collision.point.x = 1.1  #to avoid premature reenterance of collision avoidance
				self.collision.point.y = 0.4
				#if(self.out_of_bounds() == True):
				#	return

###########################################################


            # robot out of bounds procedure
#			if(self.out_of_bounds() == True):
#				return

			# vicon connection loss procedure
			if(self.vicon_connection_loss(last_seq_vicon) == True):
				return

			last_seq_vicon = self.pose.header.seq
			# Publish at the desired rate.
			self.rate.sleep()

		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		print('SUCCESS Position')
		print(self.pose)
		while(abs(self.goal_angle()) >= orientation_tolerance):
			vel_msg.angular.z = self.goal_angular_vel()
			# Publishing our vel_msg
			self.velocity_publisher.publish(vel_msg)
			# Publish at the desired rate.
			self.rate.sleep()

		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		# If we press control + C, the node will stop.
		print('SUCCESS Orientation')
		self.status = 'success'
		return self.status
		#rospy.spin()

	def out_of_bounds(self):
    		if((self.pose.transform.translation.x>9.7) or (self.pose.transform.translation.x<-7.4) or (self.pose.transform.translation.y<-3.4) or (self.pose.transform.translation.y>2.5)):
			self.vel_msg.linear.x = 0
			self.vel_msg.angular.z = 0
			self.velocity_publisher.publish(self.vel_msg)
			print('Robot exit the limit')
			self.status= 'Robot exit the limit'
			return True
		else:
			return False

	def vicon_connection_loss(self, last_seq_vicon):
		timer = time.time()
		while(self.pose.header.seq == last_seq_vicon):
			last_seq_vicon = self.pose.header.seq
			while((time.time() - timer) >= self.time_limit):
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = 0
				self.velocity_publisher.publish(self.vel_msg)
				print('Robot Vicon connection lost')
				timer = time.time()
				self.status= 'Robot Vicon connection lost'
				#return self.status
				return True
		return False		

	def rotate_in_place(self):
		rotation_speed = 0.9
		readings_num = 0
		last_collision_point = self.collision.point.x
		#print('Last collision point ', last_collision_point)
		enter_flag = True
		while(True):
			if(enter_flag == True):
				if(self.collision.point.y < 0):
					rotation_speed = -1 * rotation_speed
					enter_flag = False
				else:
					rotation_speed = rotation_speed
					enter_flag = False
			self.vel_msg.angular.z = rotation_speed
			self.velocity_publisher.publish(self.vel_msg)
			#print('rotating: readings number ', readings_num)
			#print('Last collision point ', last_collision_point)
			#print('current collision point ', self.collision.point.x)
			if(round(last_collision_point, 5) == round(self.collision.point.x, 5)):
				if(readings_num >= 1):
					break
				else:
					readings_num = readings_num + 1
			last_collision_point = self.collision.point.x
			rospy.sleep(0.12)
		self.vel_msg.angular.z = 0.0
		self.velocity_publisher.publish(self.vel_msg)
    		

	def drive(self):
		start_time = time.time()
		while(True):
			if((time.time() - start_time) > 0.8):
				break
			else:
				self.vel_msg.linear.x = 0.6
				self.velocity_publisher.publish(self.vel_msg)
		self.vel_msg.linear.x = 0.0
		self.velocity_publisher.publish(self.vel_msg)
		 
	#final rotation to goal orientation
	def grab_goal(self, request):
		print(request)
		self.string = 'success'
		self.goal = request.goal
		self.path_planning_goal.translation.x = self.goal.transform.translation.x
		self.path_planning_goal.translation.y = self.goal.transform.translation.y
		rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps')
		reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
		reset_costmaps()
		rospy.sleep(0.7)
		print('costmaps cleared')
		self.call_path_follow_server()
		print(self.string)
		return self.string

	def move_server(self):
		s = rospy.Service('drive', drive, self.grab_goal)
		rospy.spin()
		return

if __name__ == '__main__':
	try:
	 x = TurtleBot()
	 success = x.move_server()
	 #x.move2goal()
	except rospy.ROSInterruptException:
	 pass
