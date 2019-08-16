from drivetrain import DriveTrain
from shooter import Shooter
from arm import Arm
import wpilib.buttons
import wpilib.drive
import wpilib
import os
import navx
import ctre

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.driver_stick   = wpilib.Joystick(0)
        self.codriver_stick = wpilib.Joystick(1)

        self.left_shooter_motor      = ctre.WPI_TalonSRX(0)
        self.left_drivetrain_motor   = ctre.WPI_TalonSRX(1)
        self.right_drivetrain_motor  = ctre.WPI_TalonSRX(2)
        self.right_shooter_motor     = ctre.WPI_TalonSRX(3)
        
        self.intake_motor    = wpilib.Spark(8)
        self.arm_pivot_motor = wpilib.Spark(6)
        self.arm_lock_motor  = wpilib.Spark(5)

        self.timer      = wpilib.Timer()
        #self.navx       = navx.AHRS.create_spi()

        self.duration   = 20

        self.drivetrain = DriveTrain(self.left_drivetrain_motor, self.right_drivetrain_motor)
        self.shooter    = Shooter(self.intake_motor, self.left_shooter_motor, self.right_shooter_motor)
        self.arm        = Arm(self.arm_pivot_motor, self.arm_lock_motor)


    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        self.teleopPeriodic()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        start = self.timer.getMsClock()
        
        self.drivetrain.drive(self.driver_stick)
        self.shooter.shoot(self.codriver_stick)
        self.arm.lift(self.codriver_stick)

        """
        wpilib.SmartDashboard.putNumber("NavX", self.navx.getAngle())
        """
        wpilib.SmartDashboard.putNumber("Left drivetrain speed", self.left_drivetrain_motor.getQuadratureVelocity())
        wpilib.SmartDashboard.putNumber("Right drivetrain speed", self.right_drivetrain_motor.getQuadratureVelocity())
        """
        wpilib.SmartDashboard.putNumber("Left shooter position", self.left_shooter_motor.getQuadraturePosition())
        wpilib.SmartDashboard.putNumber("Right shooter position", self.right_shooter_motor.getQuadraturePosition())
        """
        wpilib.SmartDashboard.putNumber("Left shooter", self.left_shooter_motor.getQuadratureVelocity())
        wpilib.SmartDashboard.putNumber("Right shooter", self.right_shooter_motor.getQuadratureVelocity())
        """
        wpilib.SmartDashboard.putNumber("Left shooter RPM error", self.left_shooter_motor.getClosedLoopError(0) * 600/4096)
        wpilib.SmartDashboard.putNumber("Right shooter RPM error", self.right_shooter_motor.getClosedLoopError(0) * 600/4096)
        """

        self.duration = self.timer.getMsClock() - start

        wpilib.SmartDashboard.putNumber("Loop duration", self.duration)


if __name__ == "__main__":
    wpilib.run(Robot)