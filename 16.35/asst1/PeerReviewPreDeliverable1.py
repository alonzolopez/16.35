import unittest
import numpy as np
class Control(object):
	def __init__(self, s, omega):
		self.s = np.clip(s, 5, 10)
		self.omega = np.clip(omega, -np.pi/4, np.pi/4)


	def getSpeed(self):
		return self.s

	def getRotVel(self):
		return self.omega


class GroundVehicle(object):
	def __init__(self, pose, s, omega):
		self.control = Control(s, omega)
		self.pose = (np.clip(pose[0], 0, 100), np.clip(pose[1], 0, 100), np.clip(pose[2], -np.pi, np.pi))

	def getPosition(self):
		return self.pose

	def getVelocity(self):
		speed = self.control.getSpeed()
		theta = self.pose[2]

		xdot = speed*np.cos(theta)
		ydot = speed*np.sin(theta)

		return [xdot, ydot, self.control.getRotVel()]

	def setPosition(pose):
		self.pose = (np.clip(pose[0], 0, 100), np.clip(pose[1], 0, 100), np.clip(pose[2], -np.pi, np.pi))

	def setVelocity(vel):
		xdot = vel[0]
		ydot = vel[1]
		omega = np.clip(vel[2], -np.pi/4, np.pi/4)

		speed = np.sqrt((xdot**2+ydot**2))

		if speed < 5:
			deficiency = 5 - speed
			theta = np.arctan(vel[0]/vel[1])
			xdot = vel[0] + deficiency*np.cos(vel[0])
			ydot = vel[1] + deficiency*np.sin(vel[1])

		elif speed > 10:
			extra = speed - 10
			theta = np.arctan(vel[0]/vel[1])
			xdot = vel[0] - extra*np.cos(vel[0])
			ydot = vel[1] - extra*np.sin(vel[1])

		speed = np.sqrt((xdot**2+ydot**2))

		self.control = (speed, omega)

	def controlVehicle(c):
		self.control = c
		#need to update self.velocity as well, but not sure what values to give it for xdot and ydot

	def updateState(sec, msec):
		x = 
		y = 
		theta = 
		s = 
		xdot = 
		ydot = 
		omega = 


class TestControl(unittest.TestCase):

	def testSpeedInputMatchesOutput(self):
		c = Control(7, 0)
		self.assertEquals(c.getSpeed(), 7, "Speed Output does not match input")

	def testRotVelInputMatchesOutput(self):
		c = Control(7, np.pi/8)
		self.assertEquals(c.getRotVel(), np.pi/8, "RotVel Output does not match input")

	def testSpeedAboveMax(self):
		c = Control(15, 0)
		self.assertTrue(c.getSpeed() <= 10 and c.getSpeed() >= 5, "Speed above Max raises error")

	def testSpeedBelowMin(self):
		c = Control(2, 0):
		self.assertTrue(c.getSpeed() <= 10 and c.getSpeed() >= 5, "Speed below Min raises error")

	def testRotVelAboveMax(self):
		c = Control(7, np.pi)
		self.assertTrue(c.getRotVel() <= np.pi/4 and c.getRotVel() >= -np.pi/4, "Rot Vel above max raises error")

	def testRotVelBelowMin(self):
		c = Control(7, -np.pi)
		self.assertTrue(c.getRotVel() <= np.pi/4 and c.getRotVel() >= -np.pi/4, "Rot Vel below min raises error")

class TestGroundVehicle(unittest.TestCase):

	def setUp(self):
		self.gv1 = GroundVehicle([150, 150, 2*np.pi], 7, 0)
		self.gv2 = GroundVehicle([-50, -50, -2*np.pi], 7, 0)

	def testInputWithinBounds(self):
		x1 = self.gv1.getPosition()[0]
		x2 = self.gv2.getPosition()[0]
		y1 = self.gv1.getPosition()[1]
		y2 = self.gv2.getPosition()[1]
		theta1 = self.gv1.getPosition()[2]
		theta2 = self.gv2.getPosition()[2]
		xCheck = x1 <= 100 and x1 >= 0 and x2 <= 100 x2 >= 0
		yCheck = y1 <= 100 and y1 >= 0 and y2 <= 100 and y2 >= 0
		thetaCheck = theta1 >= -np.pi and theta1 <= np.pi and theta2 >= -np.pi and theta2 <= np.pi
		self.assertTrue(xCheck and yCheck and thetaCheck, "Input Bounds Violated")

	def testSetPosition(self):
		self.gv1.setPosition([150, 150, 2*np.pi], 7, 0)
		self.gv2.setPosition([-50, -50, -2*np.pi], 7, 0)
		x1 = self.gv1.getPosition()[0]
		x2 = self.gv2.getPosition()[0]
		y1 = self.gv1.getPosition()[1]
		y2 = self.gv2.getPosition()[1]
		theta1 = self.gv1.getPosition()[2]
		theta2 = self.gv2.getPosition()[2]
		xCheck = x1 <= 100 and x1 >= 0 and x2 <= 100 x2 >= 0
		yCheck = y1 <= 100 and y1 >= 0 and y2 <= 100 and y2 >= 0
		thetaCheck = theta1 >= -np.pi and theta1 <= np.pi and theta2 >= -np.pi and theta2 <= np.pi
		self.assertTrue(xCheck and yCheck and thetaCheck, "setPosition Failed; Violated Bounds")

	def testSetVelocity(self):
		self.gv1.setVelocity([10, 10, np.pi])
		self.gv2.setVelocity([1, 1, -np.pi])
		x1 = self.gv1.getVelocity()[0]
		x2 = self.gv2.getVelocity()[0]
		y1 = self.gv1.getVelocity()[1]
		y2 = self.gv2.getVelocity()[1]
		speed1 = np.sqrt(x1**2 + y1**2)
		speed2 = np.sqrt(x2**2 + y2**2)
		theta1 = self.gv1.getVelocity()[2]
		theta2 = self.gv2.getVelocity()[2]
		speedCheck = speed1 <= 10 and speed1 >= 5 and speed2 <= 10 and speed2 >= 5
		thetaCheck = theta1 >= -np.pi/4 and theta1 <= np.pi/4 and theta2 >= -np.pi/4 and theta2 <= np.pi/4
		self.assertTrue(xCheck and yCheck and thetaCheck, "setVelocity Failed; Violated Bounds")

	def testControlVehicle(self):
		c1 = Control(15, np.pi)
		c2 = Control(2, -np.pi)

		self.gv1.controlVehicle(c1)
		self.gv2.controlVehicle(c2)

		vel1 = self.gv1.getVelocity()
		vel2 = self.gv2.getVelocity()

		speed1 = np.sqrt(vel1[0]**2 + vel1[1]**2)
		speed2 = np.sqrt(vel2[0]**2 + vel2[1]**2)

		speedCheck = speed1 <= 10 and speed1 >= 5 and speed2 <= 10 and speed1 >= 5
		rotvelCheck = vel[2] <= np.pi/4 and vel[2] >= -np.pi/4

		self.assertTrue(speedCheck and rotvelCheck, "controlVehicle failed, exceeded Bounds")


	def testUpdateState(self):
		pass


