import wpilib

class Arm:
    def __init__(self, arm_motor: wpilib.PWMSpeedController, arm_encoder: wpilib.Encoder):
        self.arm_motor   = arm_motor
        self.arm_encoder = arm_encoder
        self.i_acc       = 0
        self.last_error  = 0
    
    def lift(self, stick: wpilib.Joystick):
        if stick.getRawButton(10):
            #self.arm_motor.set(0.6)
            self.pid_to_position(60)
        elif stick.getRawButton(11):
            self.pid_to_position(150)
        else:
            self.arm_motor.set(0)
            self.i_acc       = 0
            self.last_error  = 0
    
    def cm_to_cimcoder_ticks(self, cm):
        return (2.566 * cm ) - 25.03
    
    def pid_to_position(self, setpoint):
        setpoint = self.cm_to_cimcoder_ticks(setpoint)
        error = setpoint - self.arm_encoder.get()
        self.i_acc += error

        kP = 4/180
        kI = 0
        kD = 0

        ctrl_effort = (kP * error) + (kI * self.i_acc) + (kD * (error - self.last_error))

        if(ctrl_effort > 1):
            ctrl_effort = 1
        if(ctrl_effort < -1):
            ctrl_effort = -1

        self.arm_motor.set(ctrl_effort)

        wpilib.SmartDashboard.putNumber("Wanted setpoint", setpoint)
        wpilib.SmartDashboard.putNumber("Setpoint error", error)
        wpilib.SmartDashboard.putNumber("Control effort", ctrl_effort)

        self.last_error = error
