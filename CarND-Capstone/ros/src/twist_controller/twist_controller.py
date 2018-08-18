from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

MIN_VELOCITY = 0.1


class Controller(object):
    def __init__(self, VehicleMass, DecelerationLimit, 
                 WheelRadius, WheelBase, SteerRatio, 
                 MaxLateralAcceleration, MaxSteeringAngle):
        # TODO:
        self.yaw_controller = YawController(WheelBase, SteerRatio, MIN_VELOCITY,
                                            MaxLateralAcceleration, MaxSteeringAngle)

        Kp = 1.0
        Ki = 0.5
        Kd = 0.03
        MinThrottle = 0.0
        MaxThrottle = 0.2
        self.ThrottleController = PID(Kp, Ki, Kd, MinThrottle, MaxThrottle)

        Tau = 0.5  
        SamplingTime = 0.02
        self.LowpassFilter = LowPassFilter(Tau, SamplingTime)

        self.VehicleMass = VehicleMass
        self.DecelerationLimit = DecelerationLimit
        self.WheelRadius = WheelRadius

        self.CurrentTime = rospy.get_time()

    def Control(self, DesiredVelocity, AngularVelocity, CurrentLinearVelocity, dbw_enabled):
        # TODO:
        if not dbw_enabled:
            self.ThrottleController.reset()
            return 0., 0., 0.

        CurrentLinearVelocity = self.LowpassFilter.filt(CurrentLinearVelocity)

        Steer = self.yaw_controller.get_steering(DesiredVelocity, AngularVelocity,
                                                 CurrentLinearVelocity)

        LinearVelocityError = DesiredVelocity - CurrentLinearVelocity

        CurrentTime = rospy.get_time()
        dt = CurrentTime - self.CurrentTime
        self.CurrentTime = CurrentTime

        Throttle = self.ThrottleController.step(LinearVelocityError, dt)
        Brake = 0. # Torque in N*m

        if DesiredVelocity == 0. and CurrentLinearVelocity < MIN_VELOCITY:
            Throttle = 0.
            Brake = 700  # Hold car in rest if stopped at front of light. 

        elif Throttle < 0.1 and LinearVelocityError < 0.:
            Throttle = 0.0
	    Deceleration = max(LinearVelocityError, self.DecelerationLimit)
            Brake = abs(Deceleration) * self.VehicleMass * self.WheelRadius
        return Throttle, Brake, Steer
