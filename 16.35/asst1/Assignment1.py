import unittest
import numpy as np

dt = 0.01 #sec
class Control(object):
	def __init__(self, s, omega):

		if not (5 <= s <= 10):
			raise ValueError("s is out of bounds. Please enter a value in the range 5 < s < 10.".format(s))
		if not (-np.pi/4 <= omega <= np.pi/4):
			raise ValueError("omega is out of bounds. Please enter a value in the range -pi/4 < omega < pi/4.".format(omega))
		self.s = s
		self.omega = omega


	def getSpeed(self):
		return self.s

	def getRotVel(self):
		return self.omega


class GroundVehicle(object):
	def __init__(self, pose, s, omega):
		x, y, theta = pose
		if not (0 <= x <= 100):
			raise ValueError("X position is out of bounds. Must be within the range 0 < X < 100".format(x))
		if not (0 <= y <= 100):
			raise ValueError("Y position is out of bounds. Must be within the range 0 < Y < 100".format(y))
		if not (-np.pi <= theta <= np.pi):
			raise ValueError("Theta position is out of bounds. Must be within the range 0 < Theta < 100".format(theta))
		if not (5 <= s <= 10):
			raise ValueError("S is out of bounds. Please enter a value in the range 5 < S < 10.".format(s))
		if not (-np.pi/4 <= omega <= np.pi/4):
			raise ValueError("omega is out of bounds. Please enter a value in the range -pi/4 < omega < pi/4.".format(omega))
		self.control = Control(s, omega)
		self.pose = pose

	def getPosition(self):
		return self.pose

	def getVelocity(self):
		speed = self.control.getSpeed()
		theta = self.pose[2]
		print(speed)

		xdot = speed*np.sin(theta)
		ydot = speed*np.cos(theta) 
		print(ydot)

		return [xdot, ydot, self.control.getRotVel()]

	def setPosition(self,pose):
		theta = pose[2]
		if theta < 0:
			theta = theta % (-2*np.pi)
			if theta < -np.pi:
				theta += 2*np.pi
		elif theta > 0:
			theta = theta % (2*np.pi)
			if theta > np.pi:
				theta -= 2*np.pi
		self.pose = (np.clip(pose[0], 0, 100), np.clip(pose[1], 0, 100), theta)

	def setVelocity(self,vel):
		xdot = vel[0]
		ydot = vel[1]
		omega = vel[2]

		speed = np.sqrt((xdot**2+ydot**2))
		speed = np.clip(speed, 5, 10)

		if not (-np.pi/4 <= omega <= np.pi/4):
			raise ValueError("RotVel out of bounds")

		self.control = Control(speed, omega)

	def controlVehicle(self,c):
		speed = c.getSpeed()
		theta = self.getPosition()[2]
		xdot = speed*np.cos(theta)
		ydot = speed*np.sin(theta)
		omega = c.getRotVel()
		self.setVelocity((xdot, ydot, omega))

	def updateState(self,sec, msec):
		t = sec + 0.001*msec
		if t < 0: 
			raise ValueError("time cannot be negative")
		xinit, yinit, thetainit = self.getPosition()
		xdotinit, ydotinit, omegainit = self.getVelocity()
		s = self.control.getSpeed()
		r = s/omegainit
		dtheta = omegainit*t

		xfin = xinit + r*(np.sin(thetainit+dtheta) - np.sin(thetainit))
		yfin = yinit - r*(np.cos(thetainit-dtheta) - np.cos(thetainit))
		thetafin = (thetainit + dtheta) % (2*np.pi)

		if thetafin < 0:
			thetafin = thetafin % (-2*np.pi)
			if thetafin < -np.pi:
				thetafin += 2*np.pi
		elif thetafin > 0:
			thetafin = thetafin % (2*np.pi)
			if thetafin > np.pi:
				thetafin -= 2*np.pi

		self.setPosition((xfin,yfin,thetafin))

class TestControl(unittest.TestCase):

	def testSpeed(self):
		c = Control(7, 0)
		self.assertEquals(c.getSpeed(), 7, "Speed Output does not match input")

	def testUpperSpeedBound(self):
		try:
			Control(10,0)
		except ValueError:
			self.fail("Upper speed bound not inclusive")

	def testLowerSpeedBound(self):
		try:
			Control(5,0)
		except ValueError:
			self.fail("Lower speed bound not inclusive")

	def testAboveSpeedBound(self):
		self.assertRaises(ValueError, Control, 11,0)

	def testBelowSpeedBound(self):
		self.assertRaises(ValueError, Control, 4,0)

	def testRotVel(self):
		c = Control(7,0)
		self.assertEquals(c.getRotVel(),0, "RotVel output does not match input")

	def testRotVelInputMatchesOutput(self):
		c = Control(7, np.pi/8)
		self.assertEquals(c.getRotVel(), np.pi/8, "RotVel Output does not match input")

	def testRotVelMax(self):
		try:
			Control(7,np.pi/4)
		except ValueError:
			self.fail("RotVel upper bound not inclusive")

	def testRotVelMin(self):
		try: 
			Control(7, -np.pi/4)
		except ValueError:
			self.fail("RotVel lower bound not inclusive")

	def testRotVelAboveMax(self):
		self.assertRaises(ValueError, Control, 7, np.pi)

	def testRotVelBelowMin(self):
		self.assertRaises(ValueError, Control, 7, -np.pi)
		

class TestGroundVehicle(unittest.TestCase):
	
	def setUp(self):
		self.gv1 = GroundVehicle([50,50,0], 7, 0)

	def testAcceptableConstructor(self):
		try: 
			GroundVehicle([50,50,0], 7, 0)
		except ValueError:
			self.fail("ValueError was raised where it should not have")

	def testXAtUpperBound(self):
		try:
			GroundVehicle([100,50,0], 7, 0)
		except ValueError:
			self.fail("X upper bound not inclusive")
	def testXAtLowerBound(self):
		try: 
			GroundVehicle([0,50,0], 7, 0)
		except ValueError:
			self.fail("X at lower bound is not inclusive")
	def testXAboveUpperBound(self):
		self.assertRaises(ValueError, GroundVehicle, [150, 50, 0], 7, 0)
	def testXBelowLowerBound(self):
		self.assertRaises(ValueError, GroundVehicle, [-10, 50, 0], 7, 0)
	def testYAtUpperBound(self):
		try: 
			GroundVehicle([50, 100, 0], 7, 0)
		except ValueError:
			self.fail("Y upper bound not inclusive")
	def testYAtLowerBound(self):
		try:
			GroundVehicle([50, 0, 0], 7, 0)
		except ValueError:
			self.fail("Y lower bound not inclusive")
	def testYAboveUpperBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, 150, 0], 7, 0)
	def testYBelowLowerBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, -10, 0], 7, 0)
	def testThetaAtUpperBound(self):
		try: 
			GroundVehicle([50, 50, np.pi], 7, 0)
		except ValueError:
			self.fail("Theta upper bound not inclusive")
	def testThetaAtLowerBound(self):
		try: 
			GroundVehicle([50, 50, -np.pi], 7, 0)
		except ValueError:
			self.fail("Theta lower bound not inclusive")
	def testThetaAboveUpperBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, 50, 2*np.pi], 7, 0)
	def testThetaBelowLowerBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, 50, -2*np.pi], 7, 0)
	def testSAtUpperBound(self):
		try: 
			GroundVehicle([50, 50, 0], 10, 0)
		except ValueError:
			self.fail("Speed upper bound not inclusive")
	def testSAtLowerBound(self):
		try: 
			GroundVehicle([50, 50, 0], 5, 0)
		except ValueError:
			self.fail("Speed lower bound not inclusive")
	def testSAboveUpperBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, 50, 0], 15, 0)
	def testSBelowLowerBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, 50, 0], 3, 0)
	def testOmegaAtUpperBound(self):
		try:
			GroundVehicle([50, 50, 0], 7, np.pi/4)
		except ValueError:
			self.fail("Omega Upper bound not inclusive")
	def testOmegaAtLowerBound(self):
		try:
			GroundVehicle([50, 50, 0], 7, -np.pi/4)
		except ValueError:
			self.fail("Omega lower bound not inclusive")
	def testOmegaAboveUpperBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, 50, 0], 7, np.pi)
	def testOmegaBelowLowerBound(self):
		self.assertRaises(ValueError, GroundVehicle, [50, 50, 0], 7, -np.pi)
	def testGetPosition(self):
		self.assertEquals(self.gv1.getPosition(), [50,50,0])
	def testGetVelocity(self):
		self.assertEquals(GroundVehicle([0,0,0],7,0).getVelocity(), [0, 7, 0])
	def testSetPositionInBounds(self):
		try:
			self.gv1.setPosition([25,25,np.pi/2])
		except ValueError:
			self.fail("ValueError should not be raised. Input within bounds")
	def testSetPositionXAboveBound(self):
		self.gv1.setPosition([150,50,0])
		self.assertEquals(self.gv1.getPosition()[0], 100, "X above bound does not clamp correctly")
	def testSetPositionXBelowBound(self):
		self.gv1.setPosition([-50,50,0])
		self.assertEquals(self.gv1.getPosition()[0], 0, "X below bound does not clamp correctly")
	def testSetPositionYAboveBound(self):
		self.gv1.setPosition([50,150,0])
		self.assertEquals(self.gv1.getPosition()[1], 100, "Y above bound does not clamp correctly")
	def testSetPositionYBelowBound(self):
		self.gv1.setPosition([50,-50,0])
		self.assertEquals(self.gv1.getPosition()[1], 0, "Y above bound does not clamp correctly")
	def testSetPositionThetaMapsDown(self):
		self.gv1.setPosition([50,50,3*np.pi/2])
		self.assertEquals(self.gv1.getPosition()[2], -np.pi/2, "Theta above bounds does not remap correctly")
	def testSetPositionThetaMapUp(self):
		self.gv1.setPosition([50,50,-3*np.pi/2])
		self.assertEquals(self.gv1.getPosition()[2], np.pi/2, "Theta below bounds does not remap correctly")
	def testSetPositionXAtMax(self):
		self.gv1.setPosition([100,50,0])
		self.assertEquals(self.gv1.getPosition()[0], 100, "setPosition x upper bound not inclusive")
	def testSetPositionXAtMin(self):
		self.gv1.setPosition([0,50,0])
		self.assertEquals(self.gv1.getPosition()[0], 0, "setPosition x lower bound not inclusive")
	def testSetPositionYAtMax(self):
		self.gv1.setPosition([50,100,0])
		self.assertEquals(self.gv1.getPosition()[1], 100, "setPosition y upper bound not inclusive")
	def testSetPositionYAtMin(self):
		self.gv1.setPosition([50,0,0])
		self.assertEquals(self.gv1.getPosition()[1], 0, "setPosition y lower bound not inclusive")
	def testSetPositionThetaAtMax(self):
		self.gv1.setPosition([50,50,np.pi])
		self.assertEquals(self.gv1.getPosition()[2], np.pi, "setPosition theta upper bound not inclusive")
	def testSetPositionThetaAtMin(self):
		self.gv1.setPosition([50,50,-np.pi])
		self.assertEquals(self.gv1.getPosition()[2], -np.pi, "setPosition theta lower bound not inclusive")
	def testSetVelocityXAboveBound(self):
		self.gv1.setPosition([50,50, np.pi/2])
		self.gv1.setVelocity([15,0,0])
		print(self.gv1.getVelocity()[0])
		print(self.gv1.getVelocity()[1])
		self.assertEquals(self.gv1.getVelocity()[1], 0, "xdot Above bound alters ydot = 0 when it should not.")
		self.assertEquals(self.gv1.getVelocity()[0], 10, "xdot above bound does not clamp correctly")
	def testSetVelocityYAboveBound(self):
		self.gv1.setPosition([50,50,0])
		self.gv1.setVelocity([0,15,0])
		self.assertEquals(self.gv1.getVelocity()[1], 10, "ydot Above bound does not clamp correctly")
		self.assertEquals(self.gv1.getVelocity()[0], 0, "ydot above bound alters xdot = 0 when it should not")
	def testSetVelocityXAndYAboveBoundThetaZero(self):
		self.gv1.setPosition([50,50,0])
		self.gv1.setVelocity([10, 10, 0])
		self.assertEquals(self.gv1.getVelocity()[1], 10, "ydot Above bound does not clamp correctly")
		self.assertEquals(self.gv1.getVelocity()[0], 0, "ydot above bound alters xdot = 0 when it should not")
	def testSetVelocityXAndYAboveBoundThetaPi4(self):
		self.gv1.setPosition([50,50,np.pi/4])
		self.gv1.setVelocity([10, 10, 0])
		self.assertEquals(self.gv1.getVelocity()[1], 10*np.cos(np.pi/4), "xdot and ydot Above bound does not set ydot correctly with theta = pi/4")
		self.assertEquals(self.gv1.getVelocity()[0], 10*np.sin(np.pi/4), "xdot and ydot above bound does not set xdot correctly with theta = pi/4")
	def testSetVelocityXAndYAboveBoundThetaNegPi4(self):
		self.gv1.setPosition([50,50,-np.pi/4])
		self.gv1.setVelocity([10, 10, 0])
		self.assertEquals(self.gv1.getVelocity()[1], 10*np.cos(-np.pi/4), "xdot and ydot Above bound does not set ydot correctly with theta = -pi/4")
		self.assertEquals(self.gv1.getVelocity()[0], 10*np.sin(-np.pi/4), "xdot and ydot above bound does not set xdot correctly with theta = -pi/4")
	def testSetVelocityXInBoundYZero(self):
		self.gv1.setPosition([50,50,np.pi/2])
		self.gv1.setVelocity([7,0,0])
		print(self.gv1.getVelocity()[0])
		print(self.gv1.getVelocity()[1])
		self.assertEquals(self.gv1.getVelocity()[1], 0, "xdot = 7 and ydot = 0 does not set ydot correctly with theta = pi/2")
		self.assertEquals(self.gv1.getVelocity()[0], 7, "xdot = 7 and ydot = 0 does not set xdot correctly with theta = pi/2")
	def testSetVelocityYInBoundXZero(self):
		self.gv1.setPosition([50,50,0])
		self.gv1.setVelocity([0,7,0])
		self.assertEquals(self.gv1.getVelocity()[1], 7, "xdot = 0 and ydot = 7 does not set ydot correctly with theta = 0")
		self.assertEquals(self.gv1.getVelocity()[0], 0, "xdot = 0 and ydot = 7 does not set xdot correctly with theta = 0")
	def testSetVelocityXAndYInBound(self):
		try:
			self.gv1.setPosition([50,50,0])
			self.gv1.setVelocity([6,6,0])
		except ValueError:
			self.fail("ValueError when it should not have been. Xdot and ydot in bounds.")
	def testSetVelocityOmegaInBounds(self):
		try:
			self.gv1.setPosition([50,50,0])
			self.gv1.setVelocity([6,6,np.pi/8])
		except ValueError:
			self.fail("ValueError raised when it should not be. Omega (and x,y) within bounds")
	def testSetVelocityOmegaAboveUpperBound(self):
		self.gv1.setPosition([50,50,0])
		self.assertRaises(ValueError, self.gv1.setVelocity, [6,6,np.pi])
	def testSetVelocityOmegaBelowLowerBound(self):
		self.gv1.setPosition([50,50,0])
		self.assertRaises(ValueError, self.gv1.setVelocity, [6,6,-np.pi])
	def testSetVelocityOmegaAtMax(self):
		try:
			self.gv1.setPosition([50,50,0])
			self.gv1.setVelocity([6,6,np.pi/4])
		except ValueError:
			self.fail("ValueError raised when it should not be. Omega at max")
	def testSetVelocityOmegaAtMin(self):
		try:
			self.gv1.setPosition([50,50,0])
			self.gv1.setVelocity([6,6,-np.pi/4])
		except ValueError:
			self.fail("ValueError raised when it should not be. Omega at min")
	def testControlVehicleSpeedRotVelWithinBounds(self):
		try:
			self.gv1.controlVehicle(Control(7,np.pi/8))
		except ValueError:
			self.fail("controlVehicle raises ValueError when it should not. s, and omega within bounds.")
	def testControlVehicleSpeedAtMax(self):
		try:
			self.gv1.controlVehicle(Control(10,np.pi/8))
		except ValueError:
			self.fail("controlVehicle raises ValueError when it should not. s max not inclusive")
	def testControlVehicleSpeedAtMin(self):
		try:
			self.gv1.controlVehicle(Control(5,np.pi/8))
		except ValueError:
			self.fail("controlVehicle raises ValueError when it should not. s min not inclusive")
	def testRotVelControlVehicleAtMax(self):
		try:
			self.gv1.controlVehicle(Control(6, np.pi/4))
		except ValueError:
			self.fail("ValueError incorrectly raised when controlVehicle sets RotVel at max.")
	def testRotVelControlVehicleAtMin(self):
		try:
			self.gv1.controlVehicle(Control(6, -np.pi/4))
		except ValueError:
			self.fail("ValueError incorrectly raised when controlVehicle sets RotVel at min.")
		
	def testUpdateState(self):
		pass
def perpendicular(a):
		#returns a vector perpendicular to a
		b = np.empty_like(a)
		b[0] = -a[1]
		b[1] = a[0]
		return b
def findReferencePoint(a1, a2, b1, b2):
	#takes in endpoints of lines a and b in the form [x,y] 
	#finds the intersection between the two lines
	a1 = np.array(a1)
	a2 = np.array(a2)
	b1 = np.array(b1)
	b2 = np.array(b2)
	da = a2-a1
	db = b2-b1
	dp = a1-b1
	dap = perpendicular(da)
	denom = np.dot(dap,db)
	num = np.dot(dap,dp)
	return ((num/denom)*db + b1)
class Simulator(object):
	def __init__(self):
		self.sec = 0
		self.msec = 0
		self.numsides = 5
		self.clock = self.sec + 0.001*self.msec
		self.polycorners = np.zeros(5)
		self.computeCorners() #fills self.polycorners
		init_angle = np.pi - (1.0*self.numsides - 2) * np.pi/(2*self.numsides) #radians
		self.gv = GroundVehicle([0,25,init_angle], 5,0)

	def getCurrentSec(self):
		return self.sec

	def getCurrentMSec(self):
		return self.msec

	def getControl(self, sec, msec):
		time = sec + 0.001*msec
		if time < 100:
			x,y,theta = self.gv.getPosition()
			b1 = [1.0*x, 1.0*y] #current position
			center = [0.0,0.0] #center of polygon
			#get some phi in the forward clockwise direction for our reference point
			phi = np.arctan(1.0*x/y) #current phi
			xdot,ydot,omega = self.gv.getVelocity()
			speed = np.sqrt((xdot**2+ydot**2))
			r = 25.0 #radius
			dphi = np.arctan(speed*dt/r)
			final_phi = phi + dphi #some phi in the future, used to obtain ref point
			c1 = [1.0*r*np.sin(final_phi),1.0*r*np.cos(final_phi)]
			a1,a2 = self.nearestPolyPoints(final_phi) #get poly points around new phi
			refpos = findReferencePoint(a1,a2,c1,center)
			theta_desired = np.pi/2 + np.arctan((a2[1]-a1[1])/(a2[0]-a2[0]))
			omega_desired = (theta-theta_desired)/dt
			omega_desired = np.clip(omega_desired, -np.pi/4, np.pi/4)
			speed = 5
			print(omega_desired)
			return Control(speed,omega_desired)
		else:
			return null

	def setNumSides(self, sides):
		self.numsides = sides
		self.computeCorners() #resets points of polygon

	def run(self):
		self.sec = 0
		self.msec = 0
		self.clock = 0
		while self.clock < 100:
			self.gv.controlVehicle(self.getControl(self.sec, self.msec))
			self.gv.updateState(0,10)
			print(self.clock)
			#increment time
			self.msec = self.msec + 10
			if self.msec >= 1000:
				self.msec = 0
				self.sec += 1
			self.clock = self.sec + 0.001*self.msec

	def main(self):
		sim = Simulator()
		sim.run()

	def computeCorners(self):
		#takes number of sides as input
		#computes points of polygon circumscribed by a circle of diameter = 50m
		#takes first point at (x,y) = (0,25)
		corners = np.zeros((self.numsides,2))
		corners[0] = [0.0, 25.0]
		
		for i in range(len(corners)):
			if i == 0:
				corners[0] = (0.0,25.0)
			else:
				phi = i*2*np.pi/self.numsides
				corners[i] = [25.0*np.sin(phi), 25.0*np.cos(phi)]
		self.polycorners = corners
	def nearestPolyPoints(self,phi):
		#takes phi (from polar) position variable of gv
		#returns next and previous polygon points from self.polycorners
		phi = phi #angle of current position
		x1,y1 = self.polycorners[1] #x,y of polygon point with index 1
		phiOne = np.arctan((1.0*x1/y1)) #angle of polygon point with index 1
		index1 = int(phi/phiOne)%self.numsides #previous index value
		index2 = index1 + 1 #next index value
		if index2 == self.numsides:
			index2 = 0 #wrap around if index2 = self.numsides
		return [self.polycorners[index1], self.polycorners[index2]]


if __name__ == '__main__':
 '''suite = unittest.TestLoader().loadTestsFromTestCase(TestControl)
    suite2 = unittest.TestLoader().loadTestsFromTestCase(TestGroundVehicle)
    unittest.TextTestRunner(verbosity=2).run(suite) 
    unittest.TextTestRunner(verbosity=2).run(suite2) '''
sim = Simulator()
sim.main()
