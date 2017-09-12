import rospy
from pid import PID
from yaw_controller import YawController
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, accel_limit, wheel_radius, wheel_base,
                                     steer_ratio, max_lat_accel, max_steer_angle):
        self.velocity_pid = PID(0.2, 0.004, 3, mn=decel_limit, mx=accel_limit)
        self.steer_pid = PID(0.1, 0.004, 3, mn=-max_steer_angle,
                             mx=max_steer_angle)
        self.yawController = YawController(wheel_base, steer_ratio, 5,
                                           max_lat_accel, max_steer_angle)
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        pass

    def control(self, twist, velocity, time_diff):
        velocity_cte = twist.twist.linear.x - velocity.twist.linear.x
        linear_acceleration = self.velocity_pid.step(velocity_cte, time_diff)
        steer = self.yawController.get_steering(twist.twist.linear.x,
                                                twist.twist.angular.z,
                                                velocity.twist.linear.x)

        if linear_acceleration > 0:
            throttle = linear_acceleration
            brake = 0
        else:
            throttle = 0
            brake = linear_acceleration * self.vehicle_mass * self.wheel_radius
        return throttle, brake, steer
