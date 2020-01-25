# Subsystems
  - Intake
  - Feeder
  - Shooter
  - Winch
  - Telescopic Mast
  - WOF
  - Drivetrain

### Intake
This can fold inside and outside of the frame using a motor. The mecanum wheels spin the ball towards the middle and then omni wheels suck it into the middle.
   - Motors
     - 1 775
     - 1 Bag motor
   - Speed Controllers
     - 2 Victor SP
   - Sensors
     - maybe a limit switch? to see when it is fully inside of the bot
   - Main Methods
     - run(double power)
       - runs the feeder at power. Positive power will intake the ball.
     - extend()
       - makes the intake go the intake outside of frame
     - retract()
       - make the intake go inside of the frame. 
### Feeder
The feeder spins while the intake spins. Balls will hit a piston hard-stop at the top before hitting the flywheel. flywheel spins up then belts are ran in reverse for a moment then piston retracts then belts feed the ball to the shooter. If the shooter is at the speed it should be, it feeds a new ball to the shooter until it is out of balls. Once it is out of balls, the Shooter's PID is disabled 
   - Motors
     - 2 775
   - Speed Controllers
     - 2 Talon SRX
   - Sensors
     - 2 encoders
     - 1 ultrasonic range finder
   - Main Methods
     - run1ball()
       - runs the feeder until the shooter's flywheel slows down (due to it shooting the ball that the feeder fed it), then stops.
     - run(double velocity)
       - runs the feeder at velocity. velocity can be negative.
     - run(double velocity, double time)
       - runs the feeder at a velocity for a time
     - watchForBalls()
       - uses the ultrasonic range finder to count balls.
### Shooter
The flywheel spins very fast. The distance and height that the ball goes is proprotional to its speed. It is fed balls by the feeder.
   - Motors
     - 2 Neos
   - Speed Controllers
     - 2 spark max
   - Main Methods
     - calculateAngularVelocity(double distance)
       - gets distance from the camera. calculates the angular velocity.
     - setFlyWheelSpeed(double angularVelocity)
       - sets goalFlyWheelSpeed to angularVelocity. 
     - PID()
       - keeps the actual fly wheel speed at goalFlyWheelSpeed
     - disablePID()
       - disables the PID
     - enablePID()
       - enables the PID
### Winch
The winch system has 2 sides and connects to our climbing payload and pulls us up. If we run the motor the winch extends. If we run it in reverse the winch retracts.
   - Motors
     - 2 mini cims
   - Speed Controllers
     - 2 Victor SP
   - Main Methods
     - extend(double power)
       - fully extends winch by feeding power to the speed controller. Sets winchState to extended on start.
     - retract(double power)
       - fully retracts winch by feeding power to the speed controller. Sets winchState to retracted on start.
### Telescopic Mast
The mast lifts our climbing mechanism (currently a simple hook) into location on the bar. If we run the motor the mast extends. If we run it in reverse the mast retracts.
   - Motors
     - 1 775
   - Speed Controllers
     - 1 Victor SP
   - Main Methods
     - extend(double power)
       - fully extends Mast by feeding power to the speed controller. Sets mastState to extended on start.
     - retract(double power)
       - fully retracts Mast by feeding power to the speed controller. Sets mastState to retracted on start.

### WOF
wheel and motor are extended up by a piston. Then it can reach the WOF and spin it.
   - Actuators
     - 1 775
     - 1 piston
   - Speed Controllers
     - 1 Talon SRX
   - Sensors
     - 1 color sensor
     - 1 encoder
    - Main Methods
      - detectColor()
        - detects color from color sensor
      - spinToColor(colorValue)
        - spins to color by reading current color from sensor then telling the wheel to spin x amount using the encoder. Then reads current color again to see if adjustments need to be made. 
       - spinWheel()
         - attempts to spin wheel 3.5 times by counting how many times it sees a specific color
       - extend()
         - fully extends piston
       - retract()
         - fully retracts piston
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
      - tankDrive
        other drive method. 
        - Parameters (leftPower, rightPower)
        
      - shift
        - shifts.

# Commands

## Shooter related
 - intake
   - extends the piston to block the path of the ball from the feeder. runs the intake. runs the feeder at a constant velocity. 
 - spinUpFlyWheel
   - flywheel spins up to roughly correct velocity. 
 - shootAllBallsAccurate 
   - flywheel spins up then belts are ran in reverse for a moment then piston retracts then belts feed the ball to the shooter. If the shooter is at the speed it should be, it feeds a new ball to the shooter until it is out of balls. Once it is out of balls, the Shooter's PID is disabled 
- shootAllBallsFast 
   - flywheel spins up then belts are ran in reverse for a moment then piston retracts then belts feed the ball to the shooter. it feeds a new ball to the shooter until it is out of balls, without waiting for the flywheel to spin up. Or, it has much higher tolerance on the flywheel's speed. Once it is out of balls, the Shooter's PID is disabled. 
## WOF related
- extend
  - extend the piston
- retract
  - retract the piston
- colorSpin
  - do the color spin
- numberSpin
  - do the numberSpin
## Climbing related
- extendMast
  - fully extends the mast.
- retractMast
  - fully retracts the mast. (these commands can be overridden by each other, so we can miss climbing and try again)
- extendWinch
  - fully extends the winch (unspools, in case we missed)
- retractWinch
  - fully retracts the winch (spools up, to climb)
## Other
- runCompressor
- shift
