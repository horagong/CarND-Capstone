import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # DONE: Implement
        self.vehicle_mass = kwargs['vehicle_mass']
        self.wheel_radius = kwargs['wheel_radius']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.brake_deadband = kwargs['brake_deadband']
        self.fuel_capacity = kwargs['fuel_capacity']

        # if with full gass
        self.total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

        self.velocity_pid = PID(kp=3., ki=0, kd=0. #3., 0, 0.0001
                , mn=self.decel_limit, mx=self.accel_limit)
        self.velocity_lowpass = LowPassFilter(tau = 0.001, ts = 0.02)

        self.wheel_base = kwargs['wheel_base'] # yaw
        self.steer_ratio = kwargs['steer_ratio'] # yaw
        self.max_lat_accel = kwargs['max_lat_accel'] # yaw
        self.max_steer_angle = kwargs['max_steer_angle'] # yaw
        self.min_speed = 0

        self.yaw_controller = YawController(
                self.wheel_base, self.steer_ratio, self.min_speed
                , self.max_lat_accel, self.max_steer_angle)
        self.yaw_lowpass = LowPassFilter(tau = 0.001, ts = 0.02)

        self.last_time = rospy.get_time()

    def reset(self):
        self.velocity_pid.reset()

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, dbw_enabled):
        # DONE: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        curr_time = rospy.get_time()
        sample_time = curr_time - self.last_time
        self.last_time = curr_time
        error = proposed_linear_velocity - current_linear_velocity
        accel = self.velocity_pid.step(error, sample_time)
        accel = self.velocity_lowpass.filt(accel)

        steer = self.yaw_controller.get_steering(proposed_linear_velocity
            , proposed_angular_velocity, current_linear_velocity)
        steer = self.yaw_lowpass.filt(steer)

        # accelerate
        if accel > 0.:
            throttle = accel
            brake = 0.
        # decelerate
        else:
            throttle = 0.
            # brake in units of torque (N*m). 
            # The correct values for brake can be computed 
            # using the desired acceleration, weight of the vehicle, and wheel radius.
            # if proposed_linear_velocity is too small, make full brake
            if proposed_linear_velocity < 0.1:
                brake = abs(self.decel_limit) * self.total_mass * self.wheel_radius
            else:
                decel = abs(accel)
                if decel < self.brake_deadband:
                    decel = 0.
                brake = decel * self.total_mass * self.wheel_radius

        return throttle, brake, steer
