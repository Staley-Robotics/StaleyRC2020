# Subsystems
  - Intake
  - Magazine
  - Shooter
  - Winch
  - Telescopic Mast
  - WOF
  - Drivetrain

### Intake
This can fold inside and outside of the frame using a motor. The mecanum wheels spin the ball towards the middle and then 2 inch Colson wheels suck it into the middle.
   - Motors
     - 2 775 
   - Speed Controllers
     - 1 Victor SP
     - 1 Talon SRX
   - Sensors
     - encoder on joint
   - Main Methods
     - runIntake(double power)
       - runs the intake forward when given positive power, and backwards when given negative 
     - lowerIntake()
       - lowers the intake joint from its starting raised position to the lowered position where it can intake balls 
     - raiseIntake()
       - raises the intake joint from its lowered position to its raised position inside the frame  
### Magazine
The magazine spins while the intake spins. Balls will hit a piston hard-stop at the top before hitting the flywheel. Flywheel spins up then belts are ran in reverse for a moment then piston retracts then belts feed the ball to the shooter. If the shooter is at the speed it should be, it feeds a new ball to the shooter until it is out of balls. Once it is out of balls, the Shooter's PID is disabled 
   - Motors
     - 2 775
   - Speed Controllers
     - 2 Victor SP
   - Main Methods
     - extendHardStop()
       - uses a piston to stop the balls
     - retractHardStop()
       - retracts the piston that stops the balls
     - runMagazine(double magazine)
       - sets the speed of the motors
### Shooter
The flywheel spins very fast. The distance and height that the ball goes is proprotional to its speed. It is fed balls by the magazine.
   - Motors
     - 2 Neos
   - Speed Controllers
     - 2 spark max
   - Main Methods
     - updatePIDConstants()
       - resets the PID constants to original values
     - setFlyWheelSpeed(double surfaceVelocity)
        - sets goalFlyWheelSpeed to surfaceVelocity
     - getTargetFlyWheelSpeed()
        - gets the speed the flywheel is set to
     - getFlyWheelSpeed()
        - gets the current flywheel speed from the neos
     - stop()
        - turns PID off 
     - calculateSurfaceVelocity(double distance)
       - takes distance, calculates the surface velocity velocity 
     - rpmToSurfaceVelocity(rpm)
        - takes rpm and converts it into surface velocity
     - surfaceVelocityToRPM(double surface velocity)
        - takes surface velocity and converts it into RPM   
### Winch
The winch system has 2 sides and connects to our climbing payload and pulls us up. If we run the motor the winch extends. If we run it in reverse the winch retracts.
   - Motors
     - 2 775 
   - Speed Controllers
     - 2 Victor SP
   - Main Methods
     - runWinch(double motorPower)
       - gives power to the winch motors
### Telescopic Mast
The mast lifts our climbing mechanism (currently a simple hook) into location on the bar. If we run the motor the mast extends. If we run it in reverse the mast retracts.
   - Motors
     - 1 775 **Check Later**
   - Speed Controllers
     - 1 Victor SP
   - Main Methods
     - runMast(double power)
       - extends or retracts the mast depending on whether positive or negative motor power is given
### WOF
Wheel and motor are extended up by a piston. Then it can reach the WOF and spin it.
   - Actuators 
     - 1 775 
     - 1 piston
   - Speed Controllers
     - 1 Talon SRX 
   - Sensors 
     - 1 color sensor
     - 1 encoder
    - Main Methods
      - getColors()
        - retrieves color
      - setGoalColor(Color goalColor)
        - sets the goal color to the color that we need to turn the wheel to
      - runWOFSpinner(double power)
        - gives the motor power to spin the WOF
      - getCurrentColor()
        - retrieves the color that the WOF is currently on
      - raiseWOF()
        - raises the piston to raise the WOF to prepare for spinning
      - lowerWOF()
        - lowers the piston to lower the WOF
### Drivetrain
6 cim belt drive
   - Actuators
     - 6 cims
     - 2 shifters
   - Speed Controllers
     - 2 Talon SRX
     - 4 Victor SPX
   - Sensors
     - 2 SRX Mag Encoders (one for each SRX)
     - Gyro
    - Main Methods
      - worldOfTanksDrive
        teleop drive method
        - Parameters (forwardPower, reversePower, turnPower)
        - Parameters (rightTrigger, leftTrigger, leftStickXAxis) (implementation for teleop)
      - tankDrive()
        other drive method. 
        - Parameters (leftPower, rightPower)
      - stopDrive()
        - stops the robot
      - getYaw()
        - get the angle the robot is at on a horizontal plane
      - getPitch()
        - gets the angle the robot is at on a vertical plane
      - zeroYaw()
        - zeroes the yaw
      - getLeftEncoderPosition()
        - gets the distance the left encoder has gone
      - getRightEncoderPosition()
        - gets the distance the right encoder has gone
      - getLeftEncoderVelocity()  
        - gets the current velocity of the left encoder
      - getRightEncoderVelocity()
        - gets the current velocity of the left encoder
      - stepsToMeters(int steps)
        - transfers the encoder values in steps to meters
      - stepsPerDeciSecToMetersPerStep(int stepsPerDecisec)
        - transforms the encoder values in steps per decisecond to meters per second
      - metersToSteps(double meters) 
        - transforms meters back into encoder steps
      - metersPerSecToStepsPerDecisec()
        - transforms meters per second into steps per decisecond
      - zeroEncoder()
        - sets the encoder values back to zero
      - getPose()
        - retrieves the position of the robot, relative to where it started.
      - getHeading()
        - retrieves the current way the robot is facing based on the gyro
      - resetOdometry()
        - resets the pose to zero zero, and the rotation to zero
      - getTrajectoryConfig()
        - creates a new trajectory with start and end velocities, and the option of reversing
      - getPoseListFromPathWeaverJson()
        - creates a list of poses by cycling through a Pathweaver Path, and converting all of the points
      - getAutonomousCommandFromTrajectory()
        - using a trajectory it creates a ramseate command, which controlls the robot's movement during auto
      - shiftLow()
        - shifts from high gear to low gear and changes the shifter state to low
      - shiftHigh()
        - shifts from low gear to high gear and sets the shifter state to high
      - toggleShift()
        - if in high gear, shifts to low gear, if in low gear, shifts to high gear, switches shifter state
      - isLowGearOptimal()
        - determines if it is optimal to be in low gear or high gear
# Commands

## Drive train related
 - TurnToAngle
   - turns the robot to a specified angle
## Intake related
 - RunIntake
   - gives power to intake motors which makes the intake run based on the amount of power given
 - ToggleJoint
   - lowers and raises intake
## Magazine related
 - RunMagazine
   - runs the magazine forward and backward depending on power given
## Mast related
 - RunMast
   - extends and retracts the mast depending on the power given 
## Shooter related
 - ShootBallsClosedLoop
   - shoots balls only when the flywheel is within the threshold
 - ShootBallsCommandGroup
   - lines up with the target, runs the magazine backwards for 0.005 seconds, retracts the hardstop then shoots all balls
 - ShootBallsOpenLoop 
   - retracts the hardstop then shoots all the balls without having a threshold for the flywheel 
## Vision related
 - VisionYawAlign
   - turns the robot to the angle it needs to be in  from the goal to make a successful shot 
## Winch related
 - RunWinch
   - runs the winch with a power passed in and if the power is negative it runs in reverse
   - the two winch motors run opposite of each other
## WOF related
 - SpinToColor
   - has the robot spin the WOF to the given color
 - SpinToCount
   - has the robot spin the WOF 3.5 times