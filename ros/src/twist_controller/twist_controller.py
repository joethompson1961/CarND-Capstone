from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.brake_deadband = brake_deadband
        self.YawController = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.throttlePID = PID(1.0, 0.000001, 0.5, decel_limit, accel_limit)
        self.steeringPID = PID(1.75, 0.0005, 0.000, -max_steer_angle, max_steer_angle)

    def control(self, linear_velocity, angular_velocity, current_velocity):
		# TODO: Change the arg, kwarg list to suit your needs
		# Return throttle, brake, steer
        tgt_throttle = 0.05*(linear_velocity - current_velocity)
        throttle = self.throttlePID.step(tgt_throttle, 0.02) # 0.02 seconds actuator update rate (50Hz)
        # negative throttle equals braking.  Braking needs to be converted to N*m with max = 3000Nm
        brake = 0
        if throttle < -self.brake_deadband:
            brake = throttle * 3000 / decel_limit # braking range: +120Nm:+3000Nm
            throttle = 0;
        
        tgt_steer = self.YawController.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer = self.steeringPID.step(tgt_steer, 0.02) # 0.02 seconds actuator update rate (50Hz)

        return throttle, brake, steer

