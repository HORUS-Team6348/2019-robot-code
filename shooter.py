import wpilib
import ctre

class Shooter:
    def __init__(self, intake_motor: wpilib.PWMSpeedController, left_shooter: ctre.WPI_TalonSRX, right_shooter: ctre.WPI_TalonSRX):
        self.intake_motor  = intake_motor
        self.left_shooter  = left_shooter
        self.right_shooter = right_shooter

        self.left_shooter.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.left_shooter.setInverted(True)
        self.left_shooter.setSensorPhase(True)
        self.left_shooter.config_kP(0, (1023*0.1)/900, 0)
        self.left_shooter.config_kI(0, (1023*0.1)/(900*120), 0)
        self.left_shooter.config_kD(0, (1023*2)/900, 0)
        self.left_shooter.config_kF(0, 1023/26000, 0)

        self.right_shooter.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.right_shooter.setSensorPhase(True)
        self.right_shooter.config_kP(0, (1023*0.1)/900, 0)
        self.right_shooter.config_kI(0, (1023*0.1)/(900*120), 0)
        self.right_shooter.config_kD(0, (1023*2)/900, 0)
        self.right_shooter.config_kF(0, 1023/26000, 0)
    
    def shoot(self, stick: wpilib.Joystick):
        if stick.getRawButton(3):
            self.left_shooter.set(ctre.ControlMode.Velocity, 3000) #rocket bajo
            self.right_shooter.set(ctre.ControlMode.Velocity, 3000)

            if self.left_shooter.getQuadratureVelocity() >= 2500 and self.right_shooter.getQuadratureVelocity() <= -2500:
                self.intake_motor.set(1)

        elif stick.getRawButton(4): #rocket medio, cargo
            self.left_shooter.set(ctre.ControlMode.Velocity, 8000)
            self.right_shooter.set(ctre.ControlMode.Velocity, 8000)

            if self.left_shooter.getQuadratureVelocity() >= 7500 and self.right_shooter.getQuadratureVelocity() <= -7500:
                self.intake_motor.set(1)
        
        elif stick.getRawButton(6): #rocket alto
            self.left_shooter.set(ctre.ControlMode.Velocity, 17000)
            self.right_shooter.set(ctre.ControlMode.Velocity, 17000)

            if self.left_shooter.getQuadratureVelocity() >= 16500 and self.right_shooter.getQuadratureVelocity() <= -16500:
                self.intake_motor.set(1)
        
        elif stick.getRawButton(1):
            power = (-stick.getRawAxis(3)+1) * 13000

            self.left_shooter.set(ctre.ControlMode.Velocity, power)
            self.right_shooter.set(ctre.ControlMode.Velocity, power)

            if self.left_shooter.getQuadratureVelocity() >= (power - 500) and self.right_shooter.getQuadratureVelocity() <= -(power - 500):
                self.intake_motor.set(1)

        elif stick.getRawButton(2): #intake main
            self.intake_motor.set(1)
            self.left_shooter.set(ctre.ControlMode.PercentOutput, -0.3)
            self.right_shooter.set(ctre.ControlMode.PercentOutput, -0.3)

        elif stick.getRawButton(5): #regresar pelota
            self.intake_motor.set(-0.5)
            self.left_shooter.set(ctre.ControlMode.PercentOutput, -0.3)
            self.right_shooter.set(ctre.ControlMode.PercentOutput, -0.3)

        else: #nada presionado
            self.left_shooter.set(ctre.ControlMode.PercentOutput, 0)
            self.right_shooter.set(ctre.ControlMode.PercentOutput, 0)
            self.intake_motor.set(0)
            
