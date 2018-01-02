from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband,decel_limit,accel_limit,wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
		self.YawController = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)


    def control(self, linear_velocity, angular_velocity, current_velocity):
		# TODO: Change the arg, kwarg list to suit your needs
		# Return throttle, brake, steer
		throttle = 0
		throttle = 0.05*(linear_velocity - current_velocity)
		steer = self.YawController.get_steering(linear_velocity, angular_velocity, current_velocity)
		return throttle, 0., steer

