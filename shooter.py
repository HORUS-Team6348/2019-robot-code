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
        if stick.getRawAxis(3) > 0.50:
            self.left_shooter.set(ctre.ControlMode.Velocity, 6000) #rocket bajo
            self.right_shooter.set(ctre.ControlMode.Velocity, 6000)

            if self.left_shooter.getQuadratureVelocity() >= 5700 and self.right_shooter.getQuadratureVelocity() <= -5700:
                self.intake_motor.set(-1)
        
        elif stick.getPOV() == 0:
            self.left_shooter.set(ctre.ControlMode.Velocity, 6500) #rocket bajo
            self.right_shooter.set(ctre.ControlMode.Velocity, 6500)

            if self.left_shooter.getQuadratureVelocity() >= 6200 and self.right_shooter.getQuadratureVelocity() <= -6200:
                self.intake_motor.set(-1)
        
        elif stick.getPOV() == 180:
            self.left_shooter.set(ctre.ControlMode.Velocity, 5500) #rocket bajo
            self.right_shooter.set(ctre.ControlMode.Velocity, 5500)

            if self.left_shooter.getQuadratureVelocity() >= 5200 and self.right_shooter.getQuadratureVelocity() <= -5200:
                self.intake_motor.set(-1)

        elif stick.getRawAxis(2) > 0.50: #intake main
            self.intake_motor.set(-1)
            self.left_shooter.set(ctre.ControlMode.PercentOutput, -0.3)
            self.right_shooter.set(ctre.ControlMode.PercentOutput, -0.3)

        elif stick.getRawButton(5) or stick.getRawButton(6): #regresar pelota
            self.intake_motor.set(0.5)
            self.left_shooter.set(ctre.ControlMode.PercentOutput, -0.3)
            self.right_shooter.set(ctre.ControlMode.PercentOutput, -0.3)

        else: #nada presionado
            self.left_shooter.set(ctre.ControlMode.PercentOutput, 0)
            self.right_shooter.set(ctre.ControlMode.PercentOutput, 0)
            self.intake_motor.set(0)
            
