from drivetrain import DriveTrain
import pathfinder as pf
import wpilib.buttons
import wpilib.drive
import wpilib
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

        self.right_drivetrain_motor  = ctre.WPI_TalonSRX(0)
        self.left_drivetrain_motor   = ctre.WPI_TalonSRX(1)
        self.right_shooter_motor     = ctre.WPI_TalonSRX(2)
        self.left_shooter_motor      = ctre.WPI_TalonSRX(3)

        self.drivetrain = DriveTrain(self.left_drivetrain_motor, self.right_drivetrain_motor)

        self.timer = wpilib.Timer()
        self.navx  = navx.AHRS.create_spi()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous mode."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drivetrain.drive(self.driver_stick)
        wpilib.SmartDashboard.putNumber("NavX", self.navx.getAngle())


if __name__ == "__main__":
    wpilib.run(Robot)