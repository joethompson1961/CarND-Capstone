from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter_naive
from lowpass import LowPassFilter_smooth
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704   # convert miles per hour to meters per second
TIME_STEP = 0.02    # 50Hz actuator update rate = 0.02 seconds timestep
MAX_BRAKE_PRESSURE = 5000

class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.speed_lmt = rospy.get_param('/waypoint_loader/velocity')  # kph 

        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.YawController = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # throttle LPF and PID config
        T_PID_P = 0.09
        T_PID_I = 0.0075
        T_PID_D = 0.005
        T_TAU   = 1.0  # lower tau --> higher cutoff frequency (tau = 0 --> passthru filter, no cutoff)
        self.throttlePID = PID(T_PID_P, T_PID_I, T_PID_D, decel_limit, accel_limit)
        self.throttleLPF = LowPassFilter_naive(T_TAU, TIME_STEP)  # use low pass filter to smooth out high frequency actuator jitter

        rospy.loginfo('P I D TAU TIME_STEP')
        rospy.loginfo('%f, %f, %f, %f, %f', T_PID_P, T_PID_I, T_PID_D, T_TAU, TIME_STEP)

        # Configure steering LPF and PID config
        # Using P proportional to speed limit helps stablize steering at lower speeds
        # S_PID_P = 0.05      # 10kph
        # S_PID_D = 1.0       # 10 kph
        # S_TAU   = 2.0       # 10kph   lower --> higher LPF cutoff frequency (tau = 0 --> passthru filter, no cutoff)
        #
        # S_PID_P = 1.21      # 40kph
        # S_PID_D = 0.85      # 40 kph
        # S_TAU   = 5.0       # 40kph   
        S_PID_P = self.speed_lmt*0.03833 - 0.335  # convert kph to kP
        S_PID_I = 0.0001
        S_PID_D = self.speed_lmt*-0.005 + 1.05       # convert kph to kD
        S_TAU = self.speed_lmt*-0.1 + 6.0
        self.steeringPID = PID(S_PID_P, S_PID_I, S_PID_D, -max_steer_angle, max_steer_angle)
        self.steeringLPF = LowPassFilter_naive(S_TAU, TIME_STEP)  # use low pass filter to smooth out high frequency actuator jitter
        
    # Return throttle, brake, steer
    def control(self, linear_velocity, angular_velocity, current_velocity):
        throttle = linear_velocity - current_velocity
        throttle = self.throttleLPF.filt(throttle)
        throttle = self.throttlePID.step(throttle, TIME_STEP) 

        # Coasting / braking
        brake = 0.0
        if throttle <= 0.0:
            # Negative throttle equals coasting/braking. Braking needs to be converted to N*m.
            # Convert throttle to brake pressure (Nm)
            # Decel_limit is specified in G's, i.e. a multiple of 9.8 m/s/s.
            brake = throttle * 2.45 * self.decel_limit * self.vehicle_mass * self.wheel_radius
            throttle = 0.0;
     
        # Maintain brake pressure when target speed is zero and vehicle is stopped or nearly stopped
        if linear_velocity == 0.0 and abs(current_velocity) < 0.4:
            brake = self.vehicle_mass * self.wheel_radius
            throttle = 0.0;

        steer = self.YawController.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer = self.steeringLPF.filt(steer)
        steer = self.steeringPID.step(steer, TIME_STEP) # 0.02 seconds actuator update rate (50Hz)

        return throttle, brake, steer

