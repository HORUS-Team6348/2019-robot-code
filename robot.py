from drivetrain import DriveTrain
import wpilib.buttons
import wpilib.drive
import wpilib
import navx
import ctre


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.driver_stick   = wpilib.Joystick(0)
        self.codriver_stick = wpilib.Joystick(1)

        self.right_motor    = ctre.WPI_TalonSRX(0)
        self.left_motor     = ctre.WPI_TalonSRX(1)

        self.drivetrain = DriveTrain(self.left_motor, self.right_motor)

        self.auto_timer = wpilib.Timer()
        self.gyro = wpilib.ADXRS450_Gyro()
        self.navx = navx.AHRS.create_spi()
        #self.navx = navx.AHRS.create_i2c() #por si no jala con spi chavos

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous mode."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drivetrain.drive(self.driver_stick)
        wpilib.SmartDashboard.putNumber("Gyro A", self.gyro.getAngle())
        wpilib.SmartDashboard.putNumber("NavX", self.navx.getAngle())



if __name__ == "__main__":
    wpilib.run(Robot)