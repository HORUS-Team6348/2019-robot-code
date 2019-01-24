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

        self.right_motor    = ctre.WPI_TalonSRX(0)
        self.left_motor     = ctre.WPI_TalonSRX(1)

        self.drivetrain = DriveTrain(self.left_motor, self.right_motor, self)

        self.turnController  = wpilib.PIDController(0.006, 0.00015, 0.00, 0.00, self.drivetrain.get_angle_error, self.drivetrain.write_rotation, 0.02)
        self.turnController.setInputRange(-180.0, 180.0)
        self.turnController.setOutputRange(-1.0, 1.0)
        self.turnController.setAbsoluteTolerance(1.0)
        self.turnController.setContinuous(True)
        self.turnController.enable()
        
        #wpilib.SmartDashboard.putData("TurnController", self.turnController)

        self.timer = wpilib.Timer()
        self.gyro  = wpilib.ADXRS450_Gyro()
        self.navx  = navx.AHRS.create_spi()

        wpilib.SmartDashboard.putBoolean("Field oriented drive", False)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous mode."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drivetrain.drive(self.driver_stick, self.navx)
        wpilib.SmartDashboard.putNumber("Gyro A", self.gyro.getAngle())
        wpilib.SmartDashboard.putNumber("NavX", self.navx.getAngle())


if __name__ == "__main__":
    wpilib.run(Robot)