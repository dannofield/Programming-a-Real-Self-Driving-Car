from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, 
                                         fuel_capacity,
                                         brake_deadband,
                                         decel_limit,
                                         accel_limit,
                                         wheel_radius,
                                         wheel_base,                                          
                                         steer_ratio,                                         
                                         max_lat_accel,
                                         max_steer_angle,
                                         min_speed,
                                         process_rate):
        #parameters
        self.vehicle_mass = vehicle_mass
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.ts = 1.0/process_rate
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.last_time = rospy.get_time()
        
        #create throttle PID object
        kp = 0.3
        kd = 0.0
        ki = 0.1
        min = 0     # minimum throttle value
        max = 0.2   # maximum throttle value
        self.throttle_PID = PID(kp, ki, kd, min, max)
        
        #create velocity low pass object
        tau_vel = 0.5 # 1/(2pi*tau) = cutoff frequency
        
        self.vel_lpf = LowPassFilter(tau_vel, self.ts)
        
        
        #create yaw controller object
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, target_angular_velocity, target_linear_velocity, current_angular_velocity, current_linear_velocity, dbw_enabled):
        
        
        # Return throttle, brake, steer        
        
        if not dbw_enabled:
            self.throttle_PID.reset()
            return 0., 0., 0.
        
               
        current_linear_velocity = self.vel_lpf.filt(current_linear_velocity)
        linear_vel_error = target_linear_velocity - current_linear_velocity
        
        
        steering = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_PID.step(linear_vel_error, sample_time)
        brake = 0.0
        
        #rospy.logwarn("Target linear velocity: {0}".format(target_linear_velocity))
        #rospy.logwarn("Target angular velocity: {0}\n".format(target_angular_velocity))
        #rospy.logwarn("Current velocity: {0}".format(current_linear_velocity))
        #rospy.logwarn("Current angular velocity: {0}".format(current_angular_velocity))
        #rospy.logwarn("steering: {0}".format(steering))
        
        if target_linear_velocity == 0.0 and current_linear_velocity < 0.1:
            throttle = 0.0
            brake = 700
        elif throttle <.1 and linear_vel_error < 0.0:
            throttle = 0.0
            decel = max(linear_vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius   
        elif throttle < 0.05:
            throttle = 0.0
            brake = 0.0
        
        return throttle, brake, steering
