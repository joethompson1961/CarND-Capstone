from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704   # convert miles per hour to meters per second
TIME_STEP = 0.02    # 50Hz actuator update rate = 0.02 seconds timestep

T_PID_P = 1.00
T_PID_I = 0.05
T_PID_D = 0.001
T_TAU   = 0.3  # lower tau --> higher cutoff frequency (tau = 0 --> passthru filter, no cutoff)

S_PID_P = 1.65
S_PID_I = 0.085
S_PID_D = 0.0001
S_TAU   = 0.15  # lower --> higher LPF cutoff frequency (tau = 0 --> passthru filter, no cutoff)

MAX_BRAKE_PRESSURE = 5000

class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.YawController = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.throttlePID = PID(T_PID_P, T_PID_D, T_PID_I, -accel_limit, accel_limit)
        self.throttleLPF = LowPassFilter(T_TAU, TIME_STEP)  # use low pass filter to smooth out high frequency actuator jitter

        self.steeringPID = PID(S_PID_P, S_PID_D, S_PID_I, -max_steer_angle, max_steer_angle)
        self.steeringLPF = LowPassFilter(S_TAU, TIME_STEP)  # use low pass filter to smooth out high frequency actuator jitter

    # Return throttle, brake, steer
    def control(self, linear_velocity, angular_velocity, current_velocity):

        tgt_throttle = 1.0*(linear_velocity - current_velocity)
        throttle = tgt_throttle
        throttle = self.throttlePID.step(throttle, TIME_STEP) 
#        throttle = self.throttleLPF.filt(throttle)

        # Negative throttle equals braking.  Braking needs to be converted to N*m.
        brake = 0.
        if throttle < 0.0:
            # Convert throttle to brake pressure (Nm). Decel_limit is specified in G's, i.e. 9.8 m/s/s.
            brake = throttle * 9.8 * self.decel_limit * self.vehicle_mass * self.wheel_radius
            throttle = 0.0;

        # Maintain brake pressure when target speed is zero and vehicle is stopped or nearly stopped
        if linear_velocity == 0.0 and current_velocity < 0.5:
            brake = self.vehicle_mass * self.wheel_radius
            throttle = 0.0
        
        tgt_steer = self.YawController.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer = self.steeringPID.step(tgt_steer, TIME_STEP) # 0.02 seconds actuator update rate (50Hz)
        steer = self.steeringLPF.filt(steer)

        return throttle, brake, steer

