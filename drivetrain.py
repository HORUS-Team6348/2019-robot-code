import math
import wpilib
from pid import PIDController

class DriveTrain:
    def __init__(self, left_motor: wpilib.PWMSpeedController, right_motor: wpilib.PWMSpeedController):
        self.left_motor = left_motor
        self.right_motor = right_motor

    @staticmethod
    def smooth_between(min, max, degrees):
        interval = max - min
        normalized = (max - degrees) / interval
        return 2 * normalized - 1

    def get_left_motor(self, degrees, gatillo):
        if degrees <= 90:
            return gatillo
        elif degrees <= 180:
            return self.smooth_between(90, 180, degrees) * gatillo
        elif degrees <= 270:
            return -1 * gatillo
        elif degrees <= 360:
            return self.smooth_between(360, 270, degrees) * gatillo
        else:
            return 0

    def get_right_motor(self, degrees, gatillo):
        if degrees <= 90:
            return -1 * self.smooth_between(90, 0, degrees) * gatillo
        elif degrees <= 180:
            return -1 * gatillo
        elif degrees <= 270:
            return -1 * self.smooth_between(180, 270, degrees) * gatillo
        elif degrees <= 360:
            return gatillo
        else:
            return 0

    def get_left_motor_dpad(self, pad, gatillo):
        if pad == 0:
            return gatillo
        elif pad == 45:
            return gatillo
        elif pad == 90:
            return gatillo
        elif pad == 135:
            return -1 * gatillo
        elif pad == 180:
            return -1 * gatillo
        elif pad == 225:
            return 0 * gatillo
        elif pad == 270:
            return -1 * gatillo
        elif pad == 315:
            return 0 * gatillo
        else:
            return 0

    def get_right_motor_dpad(self, pad, gatillo):
        if pad == 0:
            return -1 * gatillo
        elif pad == 45:
            return 0 * gatillo
        elif pad == 90:
            return 1 * gatillo
        elif pad == 135:
            return 0 * gatillo
        elif pad == 180:
            return 1 * gatillo
        elif pad == 225:
            return 1 * gatillo
        elif pad == 270:
            return -1 * gatillo
        elif pad == 315:
            return -1 * gatillo
        else:
            return 0

    def to_degrees(self, angrad):
        degrees = math.degrees(angrad)
        if degrees < 0:
            return -degrees
        else:
            return 360 - degrees

    def stop(self):
        self.set_motors(0,0)

    def drive(self, stick: wpilib.Joystick):
        if stick.getPOV() != -1:
            self.drive_with_pad(stick)
        else:
            self.drive_with_joystick(stick)

    def set_motors(self, left_power, right_power):
        wpilib.SmartDashboard.putNumber("Left motor", left_power)
        wpilib.SmartDashboard.putNumber("Right motor", right_power)

        self.left_motor.set(left_power)
        self.right_motor.set(right_power)

    def set_motors_fixed(self, left_power, right_power):
        wpilib.SmartDashboard.putNumber("Left motor", left_power)
        wpilib.SmartDashboard.putNumber("Right motor", right_power)

        self.left_motor.set(left_power)
        self.right_motor.set(-right_power)

    def drive_with_joystick(self, stick: wpilib.Joystick):
        trigger = self.get_trigger(stick)
        x       = stick.getRawAxis(0)
        y       = stick.getRawAxis(1)

        dead_zone = 0.15

        if math.fabs(x) < dead_zone and math.fabs(y) < dead_zone:
            trigger = 0
            x = 0
            y = 0

        radians = math.atan2(y, x)
        heading = self.to_degrees(radians)

        wpilib.SmartDashboard.putNumber("Heading", heading)
        wpilib.SmartDashboard.putNumber("Trigger", trigger)

        self.drive_with_heading(heading, trigger)

    def drive_with_pad(self, stick: wpilib.Joystick):
        trigger = self.get_trigger(stick)
        dpad    = stick.getPOV()

        left_power  = self.get_left_motor(dpad, trigger)
        right_motor = self.get_right_motor(dpad, trigger)

        self.set_motors(left_power, -right_motor)

    def drive_with_heading(self, heading, trigger):
        left_power  = self.get_left_motor(heading, trigger)
        right_motor = self.get_right_motor(heading, trigger)

        self.set_motors(left_power, right_motor)

    def get_trigger(self, stick: wpilib.Joystick):
        first_trigger  = stick.getRawAxis(3)
        second_trigger = stick.getRawAxis(2)

        stateA = stick.getRawButton(1)
        stateX = stick.getRawButton(3)
        stateY = stick.getRawButton(4)

        if stateA:
            return 0.55
        elif stateX:
            return 0.60
        elif stateY:
            return 0.65
        else:
            return (first_trigger * 0.45) + (second_trigger * 0.45)
