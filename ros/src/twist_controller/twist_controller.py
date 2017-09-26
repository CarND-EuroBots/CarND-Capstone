import rospy
from pid import PID
from yaw_controller import YawController
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                 brake_deadband, fuel_capacity):
        self.velocity_pid = PID(0.5, 0.001, 0.0, mn=decel_limit, mx=accel_limit)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 5,
                                            max_lat_accel, max_steer_angle)
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY

    def reset(self):
        self.velocity_pid.reset()

    def control(self, twist, velocity, time_diff):
        velocity_cte = twist.twist.linear.x - velocity.twist.linear.x
        linear_acceleration = self.velocity_pid.step(velocity_cte, time_diff)
        steer = self.yaw_controller.get_steering(twist.twist.linear.x,
                                                 twist.twist.angular.z,
                                                 velocity.twist.linear.x)

        rospy.logwarn('Current V: {}; Target V: {}; Acceleration CMD: {}'
                      .format(velocity.twist.linear.x,
                              twist.twist.linear.x,
                              linear_acceleration))

        if linear_acceleration > 0.0:
            throttle = linear_acceleration
            brake_torque = 0.0
        else:
            throttle = 0.0
            deceleration = -linear_acceleration

            # Do not brake if too small deceleration
            if deceleration < self.brake_deadband:
                deceleration = 0.0

            # Compute brake torque, in Nm
            brake_torque = deceleration * self.total_mass * self.wheel_radius

        return throttle, brake_torque, steer
