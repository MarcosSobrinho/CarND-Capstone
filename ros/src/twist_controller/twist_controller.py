from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Params(object):
    def __init__(self):
        self.vehicle_mass = None
        self.fuel_capacity = None
        self.brake_deadband = None
        self.decel_limit = None
        self.accel_limit = None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None
        self.min_speed = None

class Controller(object):
    def __init__(self, params):
        self.params = params
        self.yaw_controller = YawController(params.wheel_base, params.steer_ratio, params.min_speed, params.max_lat_accel, params.max_steer_angle)
        
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.last_time = rospy.get_time()
        
        
    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled):
        
        if not dbw_enabled : 
            self.throttle_controller.reset()
            return 0., 0., 0.

        #rospy.loginfo("Angular vel: {0}".format(angular_vel))
        #rospy.loginfo("Target velocity: {0}".format(linear_vel))
        # rospy.loginfo("Target angular velocity: {0}".format(angular_vel))
        # rospy.loginfo("Current velocity: {0}".format(current_vel))
        
        current_vel = self.vel_lpf.filt(current_vel)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        # rospy.loginfo("Velocity Error: {0}".format(vel_error))
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)    
        
        brake = 0
        
        if linear_vel == 0.0 and current_vel < 0.1 : 
            throttle = 0.0
            brake = 400
            
        elif throttle < 0.1 and vel_error < 0.0 : 
            throttle = 0.0
            decel = max(vel_error, self.params.decel_limit)
            brake = abs(decel)*self.params.vehicle_mass*self.params.wheel_radius
            
        return throttle, brake, steering