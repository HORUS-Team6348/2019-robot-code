from pathfinder.followers import EncoderFollower
from drivetrain import DriveTrain
from shooter import Shooter
from arm import Arm
import pathfinder as pf
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
        
        self.intake_motor = wpilib.Spark(8)
        self.arm_motor    = wpilib.Spark(9)

        self.arm_encoder = wpilib.Encoder(8, 9)

        self.timer      = wpilib.Timer()
        self.navx       = navx.AHRS.create_spi()
        self.ultrasonic = wpilib.AnalogInput(4)
        
        self.ultrasonic_val = 0
        self.duration   = 20

        self.drivetrain = DriveTrain(self.left_drivetrain_motor, self.right_drivetrain_motor, self.navx)
        self.shooter    = Shooter(self.intake_motor, self.left_shooter_motor, self.right_shooter_motor)
        self.arm        = Arm(self.arm_motor, self.arm_encoder)

        self.left_points  = []
        self.right_points = []

        """

        if not wpilib.RobotBase.isSimulation():
            with open("/home/lvuser/py/paths/R_R1.left.pf1.csv", "r") as left:
                for line in left.readlines():
                    if not line.startswith("dt"):
                        args = [float(x) for x in line.split(",")]
                        self.left_points.append(pf.Segment(*args))
            with open("/home/lvuser/py/paths/R_R1.right.pf1.csv", "r") as right:
                for line in right.readlines():
                    if not line.startswith("dt"):
                        args = [float(x) for x in line.split(",")]
                        self.right_points.append(pf.Segment(*args))
        """

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        """
        self.right_follower = EncoderFollower(self.right_points)
        self.left_follower  = EncoderFollower(self.left_points)

        self.right_follower.configureEncoder(self.right_drivetrain_motor.getQuadraturePosition(), 4096, 0.1524)
        self.left_follower.configureEncoder(self.left_drivetrain_motor.getQuadraturePosition(), 4096, 0.1524)

        self.right_follower.configurePIDVA(1, 0, 0, 1/3.2, 0)
        self.left_follower.configurePIDVA(1, 0, 0, 1/3.2, 0)

        self.navx.reset()
        """

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous mode."""
        """
        l = self.left_follower.calculate(self.left_drivetrain_motor.getQuadraturePosition())
        r = self.right_follower.calculate(self.right_drivetrain_motor.getQuadraturePosition())

        wpilib.SmartDashboard.putNumber("NavX", self.navx.getAngle())

        gyro_heading = self.navx.getAngle()
        desired_heading = pf.r2d(self.left_follower.getHeading())

        angleDifference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        turn = 0.8 * (-1.0/80.0) * angleDifference * 0

        self.left_drivetrain_motor.set(ctre.ControlMode.PercentOutput, l + turn)
        self.right_drivetrain_motor.set(ctre.ControlMode.PercentOutput, - 1 * (r - turn))
        """

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        start = self.timer.getMsClock()

        ultra_read =  (self.ultrasonic.getVoltage() / ((3.3 * 10) / (1024 * 5)) * 1.254) - 5.644
        self.ultrasonic_val = (0.9 * self.ultrasonic_val) + (0.1 * ultra_read)
        
        self.drivetrain.drive(self.driver_stick, self.ultrasonic_val)
        self.shooter.shoot(self.codriver_stick)
        self.arm.lift(self.codriver_stick)

        wpilib.SmartDashboard.putNumber("NavX", self.navx.getAngle())
        wpilib.SmartDashboard.putNumber("Ultrasonic (cm)", self.ultrasonic_val)
        wpilib.SmartDashboard.putNumber("CIMcoder", self.arm_encoder.get())

        
        wpilib.SmartDashboard.putNumber("Left drivetrain position", self.left_drivetrain_motor.getQuadraturePosition())
        wpilib.SmartDashboard.putNumber("Right drivetrain position", self.right_drivetrain_motor.getQuadraturePosition())
       
        """
        wpilib.SmartDashboard.putNumber("Left shooter position", self.left_shooter_motor.getQuadraturePosition())
        wpilib.SmartDashboard.putNumber("Right shooter position", self.right_shooter_motor.getQuadraturePosition())
        
        wpilib.SmartDashboard.putNumber("Left shooter", self.left_shooter_motor.getQuadratureVelocity())
        wpilib.SmartDashboard.putNumber("Right shooter", self.right_shooter_motor.getQuadratureVelocity())
        
        wpilib.SmartDashboard.putNumber("Left shooter RPM error", self.left_shooter_motor.getClosedLoopError(0) * 600/4096)
        wpilib.SmartDashboard.putNumber("Right shooter RPM error", self.right_shooter_motor.getClosedLoopError(0) * 600/4096)
        """

        self.duration = self.timer.getMsClock() - start

        wpilib.SmartDashboard.putNumber("Loop duration", self.duration)


if __name__ == "__main__":
    wpilib.run(Robot)