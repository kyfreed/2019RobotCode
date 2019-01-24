import wpilib, ctre
from wpilib.drive import MecanumDrive


class MyRobot(wpilib.IterativeRobot):
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 3
    rearLeftChannel = 4
    frontRightChannel = 5
    rearRightChannel = 1

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        """Robot initialization function"""
        self.frontLeftMotor = ctre.WPI_TalonSRX(self.frontLeftChannel)
        self.rearLeftMotor = ctre.WPI_TalonSRX(self.rearLeftChannel)
        self.frontRightMotor = ctre.WPI_TalonSRX(self.frontRightChannel)
        self.rearRightMotor = ctre.WPI_TalonSRX(self.rearRightChannel)

        # invert the left side motors
        self.frontLeftMotor.setInverted(True)

        # you may need to change or remove this to match your robot
        self.rearLeftMotor.setInverted(True)

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )

        self.drive.setExpiration(0.1)
        self.frontLeftMotor.setSafetyEnabled(False)
        self.frontRightMotor.setSafetyEnabled(False)
        self.rearLeftMotor.setSafetyEnabled(False)
        self.rearRightMotor.setSafetyEnabled(False)

        self.stick = wpilib.Joystick(self.joystickChannel)

    def autonomousInit(self):
        self.rearLeftMotor.setQuadraturePosition(0,0)

    def autonomousPeriodic(self):
        self.frontLeftMotor.set(.5)
        self.frontRightMotor.set(.5)
        self.rearLeftMotor.set(.5)
        self.rearRightMotor.set(.5)
        
    def operatorControl(self):
        """Runs the motors with Mecanum drive."""

        self.drive.setSafetyEnabled(True)
        while self.isOperatorControl() and self.isEnabled():
            # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
            # This sample does not use field-oriented drive, so the gyro input is set to zero.
            self.drive.driveCartesian(
                self.stick.getX(), self.stick.getY(), self.stick.getZ(), 0
            )

            wpilib.Timer.delay(0.005)  # wait 5ms to avoid hogging CPU cycles


if __name__ == "__main__":
    wpilib.run(MyRobot)
