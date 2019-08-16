import wpilib

class Arm:
    def __init__(self, arm_pivot_motor: wpilib.PWMSpeedController, arm_lock_motor: wpilib.PWMSpeedController):
        self.arm_pivot_motor = arm_pivot_motor
        self.arm_lock_motor  = arm_lock_motor
        self.i_acc       = 0
        self.last_error  = 0
    
    def lift(self, stick: wpilib.Joystick):
        if stick.getRawButton(1):
            self.arm_pivot_motor.set(0.5)  
        elif stick.getRawButton(4):
            self.arm_pivot_motor.set(-0.5)
        else:
            self.arm_pivot_motor.set(0)

        if stick.getRawButton(2):
            self.arm_lock_motor.set(-0.3)  
        elif stick.getRawButton(3):
            self.arm_lock_motor.set(0.3)
        else:
            self.arm_lock_motor.set(0)

    """
    def hold_in_place(self):
        self.target = self.arm_encoder.get()
     
    def pid_to_position(self):
        error = self.target - self.arm_encoder.get()
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

        wpilib.SmartDashboard.putNumber("Wanted setpoint", self.target)
        wpilib.SmartDashboard.putNumber("Setpoint error", error)
        wpilib.SmartDashboard.putNumber("Control effort", ctrl_effort)

        self.last_error = error
    """