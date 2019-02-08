import wpilib
import math
import navx

class DriveTrain:
    def __init__(self, left_motor: wpilib.PWMSpeedController, right_motor: wpilib.PWMSpeedController):
        self.left_motor = left_motor
        self.right_motor = right_motor

    @staticmethod
    def smooth_between(min, max, degrees):
        # min (angulo menor del intervalo)  1
        # max (angulo mayor del intervalo) -1
        # degrees (angulo actual del joystick)
        interval = max - min
        normalized = (max - degrees) / interval
        return 2 * normalized - 1

    def get_left_motor(self, degrees, gatillo):
        #se asume que para avanzar derecho los dos motores se ponen en 1
        if degrees <= 90:
            return gatillo
        elif degrees <= 180:
            return self.smooth_between(180, 90, degrees) * (-1 * gatillo)
        elif degrees <= 270:
            return -1 * gatillo
        elif degrees <= 360:
            return self.smooth_between(270, 360, degrees) * (-1 * gatillo)
        else:
            return 0

    def get_right_motor(self, degrees, gatillo):
        #se asume que para avanzar derecho los dos motores se ponen en 1
        if degrees <= 90:
            return self.smooth_between(90, 0, degrees) * (-1 * gatillo)
        elif degrees <= 180:
            return (-1 * gatillo)
        elif degrees <= 270:
            return self.smooth_between(180, 270, degrees) * (-1 * gatillo)
        elif degrees <= 360:
            return gatillo
        else:
            return 0

    def to_degrees(self, radians):
        #pasa de Rad a Degrees y le suma 90 para fijar los 0 grados hacia arriba en el eje vertical, avanzando clockwise
        degrees = math.degrees(radians)
        
        if degrees < 0:
            degrees = 360 + degrees
        
        degrees += 90

        if degrees > 360:
            degrees -= 360
            
        return degrees

    def stop(self):
        self.set_motors(0,0)

    def drive(self, stick: wpilib.Joystick):
        if stick.getPOV() != -1:
            self.drive_with_pad(stick)
        else:
            self.drive_with_joystick(stick)

    def set_motors(self, left_power, right_power):
        wpilib.SmartDashboard.putNumber("Left motor", left_power)
        wpilib.SmartDashboard.putNumber("Right motor", (-1 * right_power))

        self.left_motor.set(left_power)
        self.right_motor.set(-1 * right_power)

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
        wpilib.SmartDashboard.putNumber("Power", trigger)

        self.drive_with_heading(heading, trigger)

    def drive_with_pad(self, stick: wpilib.Joystick):
        trigger = self.get_trigger(stick)
        dpad    = stick.getPOV()

        wpilib.SmartDashboard.putNumber("dpad", dpad)

        left_power  = self.get_left_motor(dpad, trigger)
        right_motor = self.get_right_motor(dpad, trigger)

        self.set_motors(left_power, right_motor)

    def drive_with_heading(self, heading, trigger):
        left_power  = self.get_left_motor(heading, trigger)
        right_motor = self.get_right_motor(heading, trigger)

        self.set_motors(left_power, right_motor)

    def get_trigger(self, stick: wpilib.Joystick):
        first_trigger  = stick.getRawAxis(3)
        second_trigger = stick.getRawAxis(2)

        return (first_trigger * 0.5) + (second_trigger * 0.5)
