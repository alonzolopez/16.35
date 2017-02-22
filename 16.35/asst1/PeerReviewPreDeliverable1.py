import numpy as np


class Control(object):

    def __init__(self, s, omega):
        # I would usually not use the underscores but because we are using
        # getters I figured that at some point we might do something more
        # complicated in which case it would be useful. 
        if not (5 <= s <= 10):
            raise ValueError("s in Control must be between 5 and 10: {}".format(s))
        if not (-np.pi/4 <= omega <= np.pi/4):
            raise ValueError("omega in Control must be between -pi/4 and pi/4: {}".format(omega))
        
        self._s = s
        self._omega = omega

    def getSpeed(self):
        return self._s

    def getRotVel(self):
        return self._omega

class GroundVehicle(object):

    def __init__(self, pose, s, omega):
        self.setPosition(pose)

        initial_control = Control(s, omega)

        self.controlVehicle(initial_control)
        
    def _floatize(self):
        for attribute in ["_x", "_y", "_theta", "_x_dot", "_y_dot", "_omega"]: #dir(self):
            try:
                val = getattr(self, attribute)
            except AttributeError:
                pass
            if isinstance(val, int):
                setattr(self, attribute, float(val))
        
    def getPosition(self):
        return self._x, self._y, self._theta

    def getVelocity(self):
        return self._x_dot, self._y_dot, self._omega
    
    def setPosition(self, pose):
        """
        pose = (x, y, theta)
        x and y must be in [0, 100]
        theta is in radians and between -pi and pi
        theta = 0 is aligned with the x-axis and positive is measured up
        """
        x, y, theta = pose

        if not ((0 <= x <= 100) and (0 <= y <= 100)):
            raise ValueError("x and y in GroundVehicle must be between 0 and 100: {:.4f} and {:.4f}".format(x, y))
        if not (-np.pi <= theta <= np.pi):
            raise ValueError("theta in GroundVehicle must be between -pi and pi: {}".format(theta))
        
        self._x = x
        self._y = y
        self._theta = theta
        self._floatize()

    def setVelocity(self, vel):
        x_dot, y_dot, omega = vel
        _, _, theta = self.getPosition()
        
        speed = np.sqrt(x_dot**2 + y_dot**2)

        # Do not need exact heading, just close enough 
        if not (abs(x_dot - speed * np.cos(theta))<1e-15 and abs(y_dot - (speed * np.sin(theta)))<1e-15):

            raise ValueError("cannot set velocity in setVelocity in a different direction than the one the vehicle is facing")
        
        if not (5 <= speed <= 10):
            raise ValueError("speed in GroundVehicle must be between 5 and 10: {}".format(speed))
        if not (-np.pi/4 <= omega <= np.pi/4):
            raise ValueError("omega in GroundVehicle must be between -pi/4 and pi/4: {}".format(omega))
        
        
        self._x_dot = x_dot
        self._y_dot = y_dot
        self._omega = omega

        self._floatize()

    def controlVehicle(self, c):
        s = c.getSpeed()
        omega = c.getRotVel()

        _, _, theta = self.getPosition()

        x_dot = s * np.cos(theta)
        y_dot = s * np.sin(theta)

        self.setVelocity((x_dot, y_dot, omega))

    def updateState(self, sec, msec):
        # This dynamics model assumes that speed remains constant throughout the updated state. 
        
        total_time = sec + msec/1000.0

        init_x, init_y, init_theta = self.getPosition()
        init_x_dot, init_y_dot, init_omega = self.getVelocity()

        s = np.sqrt(init_x_dot ** 2 + init_y_dot ** 2)

        # What if init_omega is zero? 
        k = s/init_omega
        d_theta = init_omega * total_time
        
        new_x = init_x + (k * (np.sin(init_theta + d_theta) - np.sin(init_theta)))
        new_y = init_y + (k * (-np.cos(init_theta - d_theta) + np.cos(init_theta)))
        new_theta = (init_theta + d_theta) % 2 * np.pi

        if new_theta > np.pi:
            new_theta -= 2 * np.pi

        self.setPosition((new_x, new_y, new_theta))

        new_control = Control(s, new_theta)
        self.controlVehicle(new_control)

--------------------------------------------------------------------------------------------------------------
import numpy as np
import unittest

from problemset1 import *

class TestControl(unittest.TestCase):

    def setUp(self):
        pass

    def test_speed_setting(self):
        # Should be created without throwing errors:
        try:
            Control(5, 0)
            Control(7, 0)
            Control(10, 0)
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        self.assertRaises(ValueError, Control, 4, 0)
        self.assertRaises(ValueError, Control, 11, 0)
        

    def test_invalid_angular_velocity(self):
        # Should be created without errors:
        try:
            Control(7, -np.pi/4)
            Control(7, 0)
            Control(7, np.pi/4)
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")
            
        self.assertRaises(ValueError, Control, 7, -np.pi/4-0.1)
        self.assertRaises(ValueError, Control, 7, np.pi/4+0.1)

class TestGroundVehicle(unittest.TestCase):

    def setUp(self):
        self.vehicle = GroundVehicle((50, 50, 0), 6, 0)
        

    def test_floatize(self):
        self.vehicle._x = 20
        self.vehicle._floatize()
        self.assertTrue(isinstance(self.vehicle._x, float))
        self.assertEquals(self.vehicle._x, 20.0)

    def test_setPosition(self):
        # valid setting
        try:
            self.vehicle.setPosition((75, 25, 0))
            self.vehicle.setPosition((0, 0, -np.pi))
            self.vehicle.setPosition((100, 100, np.pi))
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        # x too low
        self.assertRaises(ValueError, self.vehicle.setPosition, (-1, 50, 0))

        # x too high
        self.assertRaises(ValueError, self.vehicle.setPosition, (101, 50, 0))

        # y too low
        self.assertRaises(ValueError, self.vehicle.setPosition, (50, -1, 0))

        # y too high
        self.assertRaises(ValueError, self.vehicle.setPosition, (50, 101, 0))

        # theta too low 
        self.assertRaises(ValueError, self.vehicle.setPosition, (75, 25, -np.pi-0.1))

        # theta too high
        self.assertRaises(ValueError, self.vehicle.setPosition, (75, 25, np.pi+0.1))


    def test_setVelocity_pure_x(self):
        # valid settings
        try:
            self.vehicle.setVelocity((5, 0, 0))
            self.vehicle.setVelocity((10, 0, np.pi/4))
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        # speed too high
        self.assertRaises(ValueError, self.vehicle.setVelocity, (11, 0, 0))


        # speed too low
        self.assertRaises(ValueError, self.vehicle.setVelocity, (4, 0, 0))
        
        # ang vel too high
        self.assertRaises(ValueError, self.vehicle.setVelocity, (7, 0, -np.pi/4 - 0.1))

        # ang vel too low
        self.assertRaises(ValueError, self.vehicle.setVelocity, (7, 0, np.pi/4 + 0.1))

        # No need to test angular velocity in further test cases because angular
        # velocity setting is not dependent on heading
            
    def test_setVelocity_pure_y(self):
        try:
            self.vehicle.setPosition((50, 50, np.pi/2))

            self.vehicle.setVelocity((0, 5, 0))
            self.vehicle.setVelocity((0, 10, np.pi/4))
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        # speed too high
        self.assertRaises(ValueError, self.vehicle.setVelocity, (0, 11, 0))

        # speed too low
        self.assertRaises(ValueError, self.vehicle.setVelocity, (0, 4, 0))


    def test_setVelocity_hybrid(self):

        
        sqrt2 = np.sqrt(2)
        self.vehicle.setPosition((50, 50, np.pi/4))

        try:
            self.vehicle.setVelocity((5/sqrt2, 5/sqrt2, 0))
            self.vehicle.setVelocity((10/sqrt2, 10/sqrt2, np.pi/4))
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        # speed too high
        self.assertRaises(ValueError, self.vehicle.setVelocity, (11/sqrt2, 11/sqrt2, 0))

        # speed too low
        self.assertRaises(ValueError, self.vehicle.setVelocity, (4/sqrt2, 4/sqrt2, 0))

        

    def test_setVelocity_heading(self):
        # Valid settings already confirmed in other tests

        # heading outside of range
        self.assertRaises(ValueError, self.vehicle.setVelocity, (0, 7, 0))
        

    def test_controlVehicle(self):
        # Already tested speed limits in setVelocity, so just need to test that it correctly

        sqrt2 = np.sqrt(2)
        c = Control(10, 0)
        self.vehicle.setPosition((50, 50, np.pi/4))
        try:
            self.vehicle.controlVehicle(c)
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        x_dot, y_dot, theta_dot = self.vehicle.getVelocity()

        self.assertTrue(abs(x_dot - 10/sqrt2) < 1e15)
        self.assertTrue(abs(y_dot - 10/sqrt2) < 1e15)

        

    def test_updateState_correctness_bounds(self):
        # Need to test dynamics as well as correctness
        
        # Correctness (Should make sure that I cannot move out of bounds)
        # Out of bounds
        c = Control(10, 0)
        self.vehicle.setPosition((95, 95, 0))
        self.vehicle.controlVehicle(c)

        self.assertRaises(ValueError, self.vehicle.updateState, 2, 0)

    def test_updateState_correctness_wrapping(self):
        # Correctly wraps angles
        c = Control(5, np.pi/4)
        self.vehicle.setPosition((50, 50, 1))
        self.vehicle.controlVehicle(c)

        try:
            self.vehicle.updateState(4, 0)
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        new_x, new_y, current_heading = self.vehicle.getPosition()

        self.assertTrue(-np.pi <= current_heading <= np.pi, "heading doesn't wrap")
        
        

    def test_updateState_dynamics(self):
        # Dynamics are correct
        
        c = Control(5, np.pi/4)
        self.vehicle.setPosition((50, 50, 0))
        self.vehicle.controlVehicle(c)

        try:
            self.vehicle.updateState(4, 0)
        except ValueError:
            self.fail("ValueError raised when it shouldn't be")

        new_x, new_y, current_heading = self.vehicle.getPosition()

        # This should trace out a half circle.
        expected_x = 50
        expected_y = 2 * 20/np.pi + 50
        print new_x
        print new_y
        

        

    
class TestSimulator(unittest.TestCase):
    def setUp(self):
        pass

    

if __name__ == "__main__":
    unittest.main()

