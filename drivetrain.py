import wpilib
import math
import ctre

class DriveTrain:
    def __init__(self, left_motor: ctre.WPI_TalonSRX, right_motor: ctre.WPI_TalonSRX):
        self.left_motor  = left_motor
        self.right_motor = right_motor
        #self.navx        = navx

        self.left_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.left_motor.setSensorPhase(True)
        self.left_motor.config_kP(0, (1023 * 0.5)/4096, 0)
        self.left_motor.config_kI(0, 0, 0)
        self.left_motor.config_kD(0, 0, 0)
        self.left_motor.config_kF(0, 0, 0)

        self.left_motor.configVoltageCompSaturation(11.0, 0)
        self.left_motor.enableVoltageCompensation(True)
        self.left_motor.configVoltageMeasurementFilter(32, 0)

        self.right_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.right_motor.setInverted(True)
        self.right_motor.setSensorPhase(True)
        self.right_motor.config_kP(0, (1023 * 0.5)/4096, 0)
        self.right_motor.config_kI(0, 0, 0)
        self.right_motor.config_kD(0, 0, 0)
        self.right_motor.config_kF(0, 0, 0)

        self.right_motor.configVoltageCompSaturation(11.0, 0)
        self.right_motor.enableVoltageCompensation(True)
        self.right_motor.configVoltageMeasurementFilter(32, 0)

        self.rotation_started     = False
        self.distance_pid_started = False
        self.target_angle     = 0
        self.target_encoder_ticks = 0
        self.i_acc            = 0
        self.last_error       = 0

        self.changeFrontButtonPressed = False
        self.frontStatus = True

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
    
    def changeFront(self):
        if self.frontStatus:
            self.left_motor.setInverted(False)
            self.right_motor.setInverted(True)
        else:
            self.left_motor.setInverted(True)
            self.right_motor.setInverted(False)

     
    def drive(self, stick: wpilib.Joystick):
        if self.isRBPressed(stick) and not self.changeFrontButtonPressed:
            self.frontStatus = not self.frontStatus
            self.changeFront()
            self.changeFrontButtonPressed = True
        elif not self.isRBPressed(stick) and self.changeFrontButtonPressed:
            self.changeFrontButtonPressed = False
        """
        if self.get_joystick_button(stick) in ("X", "B"):
            if not self.rotation_started:
                self.reset_angle_pid()
                self.rotation_started = True
                if self.get_joystick_button(stick) == "X":
                    self.target_angle = self.navx.getAngle() - 90
                else:
                    self.target_angle = self.navx.getAngle() + 90
            self.rotate_to_angle(stick)
            self.reset_distance_pid()
        """
        
        if self.get_joystick_button(stick) in ("Y", "A"):
            if not self.distance_pid_started:
                self.distance_pid_started = True
                self.right_motor_offset = self.right_motor.getQuadraturePosition()
                self.left_motor_offset  = self.left_motor.getQuadraturePosition()
            
            if self.get_joystick_button(stick) == "Y":
                right_target_encoder_ticks = self.right_motor_offset - ((63-21) * 85.551)
                left_target_encoder_ticks  = self.left_motor_offset + ((63-21) * 85.551)
            else:
                right_target_encoder_ticks = self.right_motor_offset - ((130-21) * 85.551)
                left_target_encoder_ticks  = self.left_motor_offset + ((130-21) * 85.551)
            
            if self.right_motor.getQuadraturePosition() < right_target_encoder_ticks:
                self.right_motor.set(ctre.ControlMode.PercentOutput, 0)
                self.left_motor.set(ctre.ControlMode.PercentOutput, 0)
                return
            else:
                self.right_motor.set(ctre.ControlMode.Velocity, -2400)
            
            if self.left_motor.getQuadraturePosition() > left_target_encoder_ticks:
                self.right_motor.set(ctre.ControlMode.PercentOutput, 0)
                self.left_motor.set(ctre.ControlMode.PercentOutput, 0)
                return
            else:
                self.left_motor.set(ctre.ControlMode.Velocity, -2400)
        
        elif stick.getPOV() != -1:
            self.drive_with_pad(stick)  
            self.reset_angle_pid()
            self.reset_distance_pid()
        
        else:
            self.drive_with_joystick(stick)
            self.reset_angle_pid()
            self.reset_distance_pid()
    
    def reset_angle_pid(self):
        self.rotation_started = False
        self.i_acc            = 0
        self.last_error       = 0
    
    def reset_distance_pid(self):
        self.distance_pid_started = False
    
    """
    def rotate_to_angle(self, stick: wpilib.Joystick):
        error = self.navx.getAngle() - self.target_angle
        self.i_acc += error
        
        kP = 6/90
        kI = 6/(90*40)
        kD = 30/90

        ctrl_effort = (kP * error) + (kI * self.i_acc) + (kD * (error - self.last_error))

        self.set_motors(-ctrl_effort, ctrl_effort)

        wpilib.SmartDashboard.putNumber("Angle error", error)
        wpilib.SmartDashboard.putNumber("Control effort", ctrl_effort)
        
        self.last_error = error
    """
    
    def isRBPressed(self, stick: wpilib.Joystick):
        return stick.getRawButton(6)
    
    def get_joystick_button(self, stick: wpilib.Joystick):
        stateA  = stick.getRawButton(1)
        stateB  = stick.getRawButton(2)
        stateX  = stick.getRawButton(3)
        stateY  = stick.getRawButton(4)

        if stateA:
            return "A"
        elif stateB:
            return "B"
        elif stateX:
            return "X"
        elif stateY:
            return "Y"
        else:
            return False

    def set_motors(self, left_power, right_power):
        #wpilib.SmartDashboard.putNumber("Left motor", left_power)
        #wpilib.SmartDashboard.putNumber("Right motor", right_power)

        self.left_motor.set(ctre.ControlMode.PercentOutput, left_power)
        self.right_motor.set(ctre.ControlMode.PercentOutput, right_power)

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

        if not self.frontStatus:
            heading = 360 - heading

        #wpilib.SmartDashboard.putNumber("Heading", heading)
        #wpilib.SmartDashboard.putNumber("Power", trigger)

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
