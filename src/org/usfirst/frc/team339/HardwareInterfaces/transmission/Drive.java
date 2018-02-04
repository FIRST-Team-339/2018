package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.KilroyGyro;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.TransmissionType;
import org.usfirst.frc.team339.vision.VisionProcessor;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The class that controls autonomous driving functions or
 * driver-assisting functions based on sensors.
 * 
 * @author Ryan McGee
 * @written 7/26/2017
 */
public class Drive
{
// The transmission objects. Only one is used based on the transmission
// object that is input.

// The reason this is not one "TransmissionBase" object is that the drive
// functions of each type require a different number of joysticks/input
// values. Thus, inheritance is hard.

private TankTransmission tankTransmission = null;

private TractionTransmission tractionTransmission = null;

private MecanumTransmission mecanumTransmission = null;

private Encoder leftFrontEncoder = null, rightFrontEncoder = null,
        leftRearEncoder = null, rightRearEncoder = null;

private UltraSonic frontUltrasonic = null;

private UltraSonic rearUltrasonic = null;

private KilroyGyro gyro = null;

private VisionProcessor visionProcessor = null;

private final TransmissionType transmissionType;

/**
 * Creates the Drive object. If a sensor listed is not used (except for
 * encoders), set it to null.
 * 
 * 
 * @param transmission
 *            The robot's transmission object
 * @param leftFrontEncoder
 *            The left-front corner encoder
 * @param rightFrontEncoder
 *            The right-front corner encoder
 * @param leftRearEncoder
 *            The left-rear corner encoder
 * @param rightRearEncoder
 *            The right-rear corner encoder
 * @param ultrasonic
 *            The sensor that finds distance using sound
 * @param gyro
 *            A sensor that uses a spinning disk to measure rotation.
 * @param visionProcessor
 *            The camera's vision processing code, as a sensor.
 */
public Drive (TransmissionBase transmission, Encoder leftFrontEncoder,
        Encoder rightFrontEncoder,
        Encoder leftRearEncoder, Encoder rightRearEncoder,
        UltraSonic ultrasonic, KilroyGyro gyro,
        VisionProcessor visionProcessor)
{
    this.transmissionType = transmission.getType();
    this.leftFrontEncoder = leftFrontEncoder;
    this.rightFrontEncoder = rightFrontEncoder;
    this.leftRearEncoder = leftRearEncoder;
    this.rightRearEncoder = rightRearEncoder;

    this.frontUltrasonic = ultrasonic;
    this.gyro = gyro;
    this.visionProcessor = visionProcessor;

    init(transmission);
}

/**
 * Creates the Drive object. If a sensor listed is not used (except for
 * encoders), set it to null.
 * Setup for Traction drive (only 2 motors/encoders)
 * 
 * @param transmission
 *            The robot's transmission object
 * @param leftEncoder
 *            The left-side encoder
 * @param rightEncoder
 *            The right-side encoder
 * @param ultrasonic
 *            The sensor that finds distance using sound
 * @param gyro
 *            A sensor that uses a spinning disk to measure rotation.
 */
public Drive (TransmissionBase transmission, Encoder leftEncoder,
        Encoder rightEncoder, UltraSonic frontUltrasonic,
        UltraSonic rearUltrasonic, KilroyGyro gyro,
        VisionProcessor visionProcessor)
{
    this.transmissionType = transmission.getType();
    this.leftRearEncoder = leftEncoder;
    this.rightRearEncoder = rightEncoder;

    this.frontUltrasonic = frontUltrasonic;
    this.rearUltrasonic = rearUltrasonic;
    this.gyro = gyro;
    this.visionProcessor = visionProcessor;

    init(transmission);
}

private void init (TransmissionBase transmission)
{

    // Only sets the transmission if it is the same type. Other transmission
    // objects get set to null.
    switch (transmissionType)
        {
        case MECANUM:
            this.mecanumTransmission = (MecanumTransmission) transmission;
            break;
        case TANK:
            this.tankTransmission = (TankTransmission) transmission;
            break;
        case TRACTION:
            this.tractionTransmission = (TractionTransmission) transmission;
            break;
        default:
            System.out.println(
                    "There was an error setting up the DRIVE class. "
                            + "Please check the declaration for a valid transmission object.");
            break;

        }
}

/**
 * Checks if the value input is in between -1 and 1 to keep it in range for
 * motor inputs.
 * 
 * @param val
 *            The input value
 * @return The correctly ranged value
 */
private double inRange (double val)
{
    if (val > 1)
        return 1;
    else if (val < -1)
        return -1;

    return val;
}

/**
 * Gets the transmission object stored. ONLY use it for transmission.stop()
 * and transmission.driveRaw()
 * 
 * @return The current transmission object used in the Drive class
 */
public TransmissionBase getTransmission ()
{
    // based on what transmission type we're using, print out the object
    switch (transmissionType)
        {
        case MECANUM:
            return mecanumTransmission;
        case TANK:
            return tankTransmission;
        case TRACTION:
            return tractionTransmission;
        default:
            return null;
        }
}

// ================ENCODER METHODS================
/**
 * Different groups of wheels for use in encoder data collection.
 * 
 * @author Ryan McGee
 */
public enum WheelGroups
    {
/**
 * if two wheel drive then one from each side (will be called rear of that side)
 * if in tank or mecanum, all four wheels combined
 */
ALL,
/**
 * Both front and back on the left side
 */
LEFT_SIDE,
/**
 * Both front and back on the right side
 */
RIGHT_SIDE
    }

/**
 * Sets how far the robot has driven per pulse the encoder reads.
 * This value should be much lower than one, as there are usually
 * hundreds of pulses per rotation.
 * 
 * To calculate, reset the encoders and
 * push the robot forwards, say, five feet. Then count the number of pulses
 * and do: (5x12)/pulses to get this in inches.
 * 
 * @param value
 *            The encoder distance per pulse.
 * @param encoder
 *            Which encoder will be changed
 */
public void setEncoderDistancePerPulse (double value,
        TransmissionBase.MotorPosition encoder)
{
    switch (encoder)
        {
        case ALL:
            // if we're using mecanum or tank drive then set distance
            // per pulse to all the encoders
            if (transmissionType == TransmissionType.MECANUM
                    || transmissionType == TransmissionType.TANK)
                {
                leftFrontEncoder.setDistancePerPulse(value);
                rightFrontEncoder.setDistancePerPulse(value);
                leftRearEncoder.setDistancePerPulse(value);
                rightRearEncoder.setDistancePerPulse(value);
                }
            // if we're using traction drive set pulse per encoder for both
            // sides
            else if (transmissionType == TransmissionType.TRACTION)
                {
                leftRearEncoder.setDistancePerPulse(value);
                rightRearEncoder.setDistancePerPulse(value);
                }
            break;
        case LEFT_FRONT:
            // set distance per pulse to the left front encoder
            leftFrontEncoder.setDistancePerPulse(value);
            break;
        case RIGHT_FRONT:
            // set distance per pulse to the right front encoder
            rightFrontEncoder.setDistancePerPulse(value);
            break;
        case LEFT_REAR:
            // set distance per pulse to the left rear encoder
            leftRearEncoder.setDistancePerPulse(value);
            break;
        case RIGHT_REAR:
            // set distance per pulse to the right rear encoder
            rightRearEncoder.setDistancePerPulse(value);
            break;
        default:
            break;
        }
}

/**
 * Sets all the encoder's stored pulses back to zero.
 */
public void resetEncoders ()
{
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        {
        // set all four encoders back to zero
        leftFrontEncoder.reset();
        rightFrontEncoder.reset();
        leftRearEncoder.reset();
        rightRearEncoder.reset();
        }
    else if (transmissionType == TransmissionType.TRACTION)
        {
        // sets both sides encoders back to zero
        leftRearEncoder.reset();
        rightRearEncoder.reset();
        }
}

/**
 * Gets the averages of certain wheel groups. All values are the absolute value
 * to stop negative numbers from affecting the average.
 * 
 * @param encoderGroup
 * @return
 */
public double getEncoderDistanceAverage (WheelGroups encoderGroup)
{

    if (transmissionType != TransmissionType.TRACTION)
        // if we are not using traction drive then get averaged
        // distance of all the encoders
        switch (encoderGroup)
            {
            case ALL:
                // averages all the drive encoders, the two sides if
                // we're using two wheel drive and all four if mecanum or tank
                if (transmissionType == TransmissionType.MECANUM
                        || transmissionType == TransmissionType.TANK)
                    {
                    return (Math.abs(leftFrontEncoder.getDistance())
                            + Math.abs(rightFrontEncoder.getDistance())
                            + Math.abs(leftRearEncoder.getDistance())
                            + Math.abs(rightRearEncoder.getDistance()))
                            / 4.0;
                    }
                else if (transmissionType == TransmissionType.TRACTION)
                    {
                    return (Math.abs(leftRearEncoder.getDistance())
                            + Math.abs(rightRearEncoder.getDistance()))
                            / 2.0;
                    }
            case LEFT_SIDE:
                // averages left side encoders
                return (Math.abs(leftFrontEncoder.getDistance())
                        + Math.abs(leftRearEncoder.getDistance()))
                        / 2.0;
            case RIGHT_SIDE:
                // averages right side encoders
                return (Math.abs(rightFrontEncoder.getDistance())
                        + Math.abs(rightRearEncoder.getDistance()))
                        / 2.0;
            default:
                return 0.0;
            }
    // If two wheel drive only
    return (Math.abs(leftRearEncoder.getDistance())
            + Math.abs(rightRearEncoder.getDistance())) / 2.0;
}

/**
 * Gets how many ticks is on each motor controller and adds the
 * two of a side if multiple to get the total ticks per side
 * 
 * @param encoder
 *            Which encoder position to get.
 * @return
 *         Number of Ticks
 */
public int getEncoderTicks (TransmissionBase.MotorPosition encoder)
{
    // Get the ticks of either right or left side on the 2 wheel system.
    if (transmissionType == TransmissionType.TRACTION)
        {
        switch (encoder)
            {
            // these cases return the left rear Encoder
            case LEFT_FRONT:
            case LEFT_REAR:
            case LEFT:
                return leftRearEncoder.get();
            // these cases return the right rear Encoder
            case RIGHT_FRONT:
            case RIGHT_REAR:
            case RIGHT:
                return rightRearEncoder.get();
            // returns 0 to show default case was run
            default:
                return 0;
            }
        }
    // Get encoder ticks on a 4 wheel drive system.
    switch (encoder)
        {
        // returns the total ticks on the left side encoders
        case LEFT:
            return leftFrontEncoder.get() + leftRearEncoder.get();

        // returns the total ticks on the left front encoders
        case LEFT_FRONT:
            return leftFrontEncoder.get();

        // returns the total ticks on the left rear encoders
        case LEFT_REAR:
            return leftRearEncoder.get();

        // returns the total ticks on the right side encoders
        case RIGHT:
            return rightFrontEncoder.get() + rightRearEncoder.get();

        // returns the total ticks on the right front encoders
        case RIGHT_FRONT:
            return rightFrontEncoder.get();

        // returns the total ticks on the right rear encoders
        case RIGHT_REAR:
            return rightRearEncoder.get();

        // default:returns zero
        default:
            return 0;
        }
}

/**
 * Tests whether any encoder reads larger than the input length. Useful for
 * knowing
 * when to stop the robot.
 * 
 * @param length
 *            The desired length
 * @return True when any encoder is past length
 */
private boolean isAnyEncoderLargerThan (double length)
{
    // if using mecanum or tank return true if any of the encoders are
    // greater than the parameter, return true else return false
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        return (Math.abs(leftFrontEncoder.getDistance()) > length
                || Math.abs(rightFrontEncoder.getDistance()) > length
                || Math.abs(leftRearEncoder.getDistance()) > length
                || Math.abs(rightRearEncoder.getDistance()) > length);
    // else return true if either side is greater than the parameter
    return (Math.abs(leftRearEncoder.getDistance()) > length
            || Math.abs(rightRearEncoder.getDistance()) > length);

}

// ================ DRIVE METHODS ================

/**
 * Resets the Drive class's functions, in case they were cut short.
 */
public void reset ()
{
    // sets to true
    this.driveInchesInit = true;
    this.driveStraightInchesInit = true;
    this.turnDegreesInit = true;
}

/**
 * Enum for deciding whether we're braking after driving, or turning.
 * 
 * @author Ryan McGee
 *
 */
public static enum BrakeType
    {
AFTER_DRIVE, AFTER_TURN
    }

/**
 * Expected distance that it will take to stop during brake()
 * 
 * @return
 *         expected distance during brake()
 */
public double getBrakeStoppingDistance ()
{
    return (this.distanceRequiredToBrake);
} // end getBrakeStoppingDistance()

/**
 * Store the expected distance that it will take to stop during brake()
 * 
 * @param
 *            brakeStoppingDistance
 *            - new stopping distance during braking
 * @return
 *         new stored distance during brake()
 */
public double setBrakeStoppingDistance (double brakeStoppingDistance)
{
    return (this.distanceRequiredToBrake = brakeStoppingDistance);
} // end setBrakeStoppingDistance()

/**
 * Stops the robot suddenly, to prevent drifting during autonomous functions,
 * and increase the precision.
 * 
 * @param type
 *            TODO
 * 
 * @return
 *         Whether or not the robot has stopped moving.
 */
public boolean brake (BrakeType type)
{
    // prints out calling brake
    System.out.println("Calling Brake");

    // sets deadband and power to zero
    int deadband = 0;
    double power = 0;

    // if BrakeType is AFTER_DRIVE then set deadband to brakeDriveDeadband
    // and power to brakeDrivePower
    if (type == BrakeType.AFTER_DRIVE)
        {
        deadband = brakeDriveDeadband;
        power = brakeDrivePower;
        }
    // if Braketype is AFTER_TURN set the deadband to brakeTurndeadband
    // and set power to brakeTurnPower
    else if (type == BrakeType.AFTER_TURN)
        {
        deadband = brakeTurnDeadband;
        power = brakeTurnPower;
        }


    if (System.currentTimeMillis() - previousBrakeTime > INIT_TIMEOUT)
        {
        prevEncoderValues = new double[4];

        // Get the direction of the motor values on the first start.
        brakeMotorDirection[0] = (int) Math
                .signum(leftRearEncoder.getRate());
        brakeMotorDirection[1] = (int) Math
                .signum(rightRearEncoder.getRate());
        // If it's not a 2 wheel drive, get the direction of the other 2 wheels.
        if (transmissionType != TransmissionType.TRACTION)
            {
            brakeMotorDirection[2] = (int) Math
                    .signum(leftFrontEncoder.getRate());
            brakeMotorDirection[3] = (int) Math
                    .signum(rightFrontEncoder.getRate());
            }
        }

    int[] brakeDeltas = new int[4];

    brakeDeltas[0] = getEncoderTicks(MotorPosition.LEFT_REAR)
            - brakePrevEncoderVals[0];
    brakeDeltas[1] = getEncoderTicks(MotorPosition.RIGHT_REAR)
            - brakePrevEncoderVals[1];
    brakeDeltas[2] = getEncoderTicks(MotorPosition.LEFT_FRONT)
            - brakePrevEncoderVals[2];
    brakeDeltas[3] = getEncoderTicks(MotorPosition.RIGHT_FRONT)
            - brakePrevEncoderVals[3];

    brakePrevEncoderVals[0] = getEncoderTicks(MotorPosition.LEFT_REAR);
    brakePrevEncoderVals[1] = getEncoderTicks(MotorPosition.RIGHT_REAR);
    brakePrevEncoderVals[2] = getEncoderTicks(MotorPosition.LEFT_FRONT);
    brakePrevEncoderVals[3] = getEncoderTicks(
            MotorPosition.RIGHT_FRONT);

    for (int i = 0; i < brakeDeltas.length; i++)
        System.out.println("Delta " + i + ": " + brakeDeltas[i]);

    for (int i = 0; i < brakeMotorDirection.length; i++)
        System.out.println("Power " + i + ": "
                + brakeMotorDirection[i] * -brakeDrivePower);

    // See if the motors are past the deadband
    if (brakeMotorDirection[0] * brakeDeltas[0] < deadband
            && brakeMotorDirection[1]
                    * brakeDeltas[1] < deadband
            && ((transmissionType == TransmissionType.TRACTION)
                    || brakeMotorDirection[2]
                            * brakeDeltas[2] < deadband
                    || brakeMotorDirection[3]
                            * brakeDeltas[3] < deadband))
        {
        // Increase the iteration
        currentBrakeIteration++;
        }
    else
        {
        // Reset the iteration. We want x times ~in a row~.
        currentBrakeIteration = 0;
        }

    // If we have been within the deadband for x times, return true.
    if (currentBrakeIteration >= totalBrakeIterations)
        {
        currentBrakeIteration = 0;
        getTransmission().stop();
        return true;
        }

    // Set the rear wheels
    getTransmission()
            .getSpeedController(MotorPosition.LEFT_REAR)
            .set(-brakeMotorDirection[0] * power);
    getTransmission().getSpeedController(MotorPosition.RIGHT_REAR)
            .set(-brakeMotorDirection[1] * power);
    // Set the front wheels if it's the right kind of drive
    if (transmissionType != TransmissionType.TRACTION)
        {
        getTransmission().getSpeedController(MotorPosition.LEFT_FRONT)
                .set(-brakeMotorDirection[2] * power);
        getTransmission().getSpeedController(MotorPosition.RIGHT_FRONT)
                .set(-brakeMotorDirection[3] * power);
        }
    // END SET MOTORS
    this.previousBrakeTime = System.currentTimeMillis();
    return false;
}

private long previousBrakeTime = 0;

private int[] brakeMotorDirection = new int[4];

private int brakeDriveDeadband = 55; // ticks

private int brakeTurnDeadband = 14;

private int[] brakePrevEncoderVals = new int[4];

private double brakeDrivePower = .2;

private double brakeTurnPower = 1.0;

private int currentBrakeIteration = 0;

private int totalBrakeIterations = 3;

/**
 * Sets the deadband for brake()... how close to stopped we are.
 * 
 * @param ticks
 *            Ticks on the encoder, not distance.
 */
public void setBrakeDeadband (int ticks)
{
    this.brakeDriveDeadband = ticks;
}

/**
 * Sets how much the robot should send to the motors while braking
 * 
 * @param power
 *            percentage (0.0 to 1.0)
 */
public void setBrakePower (double power)
{
    this.brakeDrivePower = power;
}

/**
 * 
 * @param iterations
 */
public void setBrakeIterations (int iterations)
{
    this.totalBrakeIterations = iterations;
}


/**
 * Drives the robot a certain distance without encoder correction.
 * Not using correction increases reliability but decreases precision.
 * If one encoder fails, it will instead look for other encoders for input.
 * 
 * @param distance
 *            how far the robot should travel. Should always remain positive!
 * @param speed
 *            how fast the robot should go while traveling. Negative for
 *            backwards.
 * @return whether or not the robot has reached "distance".
 */
public boolean driveInches (int distance, double speed)
{
    // Reset encoders on initialization.
    if (this.driveInchesInit == true)
        {
        this.resetEncoders();
        this.driveInchesInit = false;
        }

    // Test if ANY encoder is past the distance.
    if (this.isAnyEncoderLargerThan(distance) == true)
        {
        this.driveInchesInit = true;
        this.getTransmission().stop();
        return true;
        }

    this.getTransmission().drive(speed, speed);
    return false;
}

private boolean driveInchesInit = true;

/**
 * Drives the robot a certain distance based on the encoder values.
 * If the robot should go backwards, set speed to be negative instead of
 * distance.
 * 
 * @param distance
 *            How far the robot should go (should be greater than 0)
 * @param speed
 *            How fast the robot should travel
 * @return Whether or not the robot has finished traveling that given distance.
 */
public boolean driveStraightInches (int distance, double speed)
{
    // Runs once when the method runs the first time, and does not run again
    // until after the method returns true.
    if (driveStraightInchesInit == true)
        {
        this.resetEncoders();
        driveStraightInchesInit = false;
        }

    // Check all encoders
    if (this.getEncoderDistanceAverage(WheelGroups.ALL) > distance)
        {
        this.getTransmission().stop();
        driveStraightInchesInit = true;
        return true;
        }

    // Drive straight if we have not reached the distance
    this.driveStraight(speed, true);

    return false;
}

private boolean driveStraightInchesInit = true;

/**
 * Strafe to a target using a mecanum transmission, and a gyro for
 * stabilization.
 * This will NOT be accurate because of mecanum's slippery properties.
 * 
 * @param inches
 *            How far we should travel
 * @param speed
 *            How fast we should travel, in decimal percentage (0.0 to 1.0)
 * @param directionDegrees
 *            In which direction we should travel, 0 being forwards, -90 for
 *            left and 90 for right.
 * @return
 *         Whether or not we have finished strafing.
 */
public boolean strafeStraightInches (int inches, double speed,
        int directionDegrees)
{
    // Wrong transmission type!
    if (this.transmissionType != TransmissionType.MECANUM)
        return true;

    // Reset the gyro and encoders on first start only
    if (strafeStraightInchesInit)
        {
        this.resetEncoders();
        this.gyro.reset();
        strafeStraightInchesInit = false;
        }

    // If we have traveled past the distance requested, then stop.
    if (this.getEncoderDistanceAverage(WheelGroups.ALL) > inches)
        {
        strafeStraightInchesInit = true;
        this.getTransmission().stop();
        return true;
        }
    // Run the rotation in a proportional loop based on the gyro.
    this.mecanumTransmission.drive(speed,
            Math.toRadians(directionDegrees),
            -(gyro.getAngle() * strafeStraightScalar));

    return false;
}

private boolean strafeStraightInchesInit = true;

private double strafeStraightScalar = .08;

/**
 * Sets the scalar for the strafeStraightInches function.
 * 
 * @param scalar
 *            A scalar, in percent per degrees added to the rotation of the
 *            mecanum code.
 */
public void setStrafeStraightScalar (double scalar)
{
    this.strafeStraightScalar = scalar;
}

/**
 * Accelerates the robot to a certain speed, then continue at the speed input
 * into the method. Allows for accelerate and driving straight directly
 * afterwards.
 * 
 * The equation is: deltaTime / (timeRequested) * speed. The code will make sure
 * that deltaTime / timeRequested does not return outside -1 to 1.
 * 
 * @param leftSpeed
 *            The target speed for the left drive motors
 * @param rightSpeed
 *            The target speed for the right drive motors
 * @param time
 *            The time period accelerated over, in seconds. When the time
 *            reaches this
 *            number, it will be running full speed.
 * 
 * @return True if we are done accelerating. It WILL continue driving after
 *         acceleration is done, at the input speed.
 */
public boolean accelerateTo (double leftSpeed, double rightSpeed,
        double time)
{
    // If we timeout, then reset all the accelerate
    if (System.currentTimeMillis() - lastAccelerateTime > INIT_TIMEOUT)
        {
        lastAccelerateTime = System.currentTimeMillis();
        accelMotorPower = accelStartingSpeed;
        }

    // main acceleration maths
    double deltaSeconds = (System.currentTimeMillis()
            - lastAccelerateTime) / 1000.0;
    accelMotorPower += deltaSeconds / time;

    // Drive the robot based on the times and speeds
    this.getTransmission().drive(
            leftSpeed * inRange(accelMotorPower),
            rightSpeed * inRange(accelMotorPower));

    // Reset the "timer"
    lastAccelerateTime = System.currentTimeMillis();

    if (accelMotorPower > 1.0)
        return true;
    // We are not done accelerating
    return false;
}

private double accelMotorPower = 0;// Power sent to each motor

private double accelStartingSpeed = .2;

private long lastAccelerateTime = 0; // Milliseconds

private double defaultAcceleration = .8;// Seconds

/**
 * Sets the initial speed of the accelerateTo motors
 * 
 * @param value
 *            Positive percentage / motor value
 */
public void setAccelStartingSpeed (double value)
{
    this.accelStartingSpeed = value;
}

/**
 * Sets the default acceleration for driveStraight
 * 
 * @param value
 *            The acceleration period, in seconds
 */
public void setDefaultAcceleration (double value)
{
    this.defaultAcceleration = .8;
}

/**
 * Drives the robot in a straight line based on encoders.
 * 
 * This works by polling the encoders every (COLLECTION_TIME) milliseconds
 * and then taking the difference from the last collection and using it as a
 * ratio to multiply times the speed.
 * 
 * This approach allows a more dynamic correction, as it only corrects as much
 * as it needs to.
 * 
 * Remember: reset the encoders before running this method.
 * 
 * @param speed
 *            How fast the robot will be moving. Correction will be better with
 *            lower percentages.
 * @param accelerate
 *            TODO
 * @param acceleration
 *            TODO
 */
public void driveStraight (double speed, boolean accelerate)
{
    // Only use the four encoders if the robot uses a four-wheel system
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        {
        // Calculate how much has changed between the last collection
        // time and now
        leftChange = (leftFrontEncoder.getDistance()
                + leftRearEncoder.getDistance())
                - prevEncoderValues[0];
        rightChange = (rightFrontEncoder.getDistance()
                + rightRearEncoder.getDistance())
                - prevEncoderValues[1];
        // Setup the previous values for the next collection run
        prevEncoderValues[0] = leftFrontEncoder.getDistance()
                + leftRearEncoder.get();
        prevEncoderValues[1] = rightFrontEncoder.getDistance()
                + rightRearEncoder.get();
        }
    else
        {
        // Calculate how much has changed between the last collection
        // time and now
        leftChange = leftRearEncoder.getDistance()
                - prevEncoderValues[0];
        rightChange = rightRearEncoder.getDistance()
                - prevEncoderValues[1];
        // Setup the previous values for the next collection run
        prevEncoderValues[0] = leftRearEncoder.getDistance();
        prevEncoderValues[1] = rightRearEncoder.getDistance();
        }

    double delta = leftChange - rightChange;
    double leftMotorVal = speed, rightMotorVal = speed;

    // We are going forwards?
    if (speed > 0)
        {
        // Left is more forwards than right?
        if (leftChange > rightChange)
            {
            leftMotorVal = speed - driveStraightConstant;
            rightMotorVal = speed;
            }
        // Right is more forwards than left?
        else if (rightChange > leftChange)
            {
            leftMotorVal = speed;
            rightMotorVal = speed - driveStraightConstant;
            }
        }
    else if (speed < 0)
    // We are going backwards?
        {
        // Left is more backwards than right?
        if (leftChange < rightChange)
            {
            leftMotorVal = speed + driveStraightConstant;
            rightMotorVal = speed;
            }
        // Right is more backwards than left?
        else if (rightChange < leftChange)
            {
            rightMotorVal = speed + driveStraightConstant;
            leftMotorVal = speed;
            }
        }

    // Changes how much the robot corrects by how off course it is. The
    // more off course, the more it will attempt to correct.

    if (accelerate)
        this.accelerateTo(leftMotorVal, rightMotorVal,
                defaultAcceleration);
    else
        this.getTransmission().drive(leftMotorVal, rightMotorVal);

}

// How much should be subtracted from the left or right side while driving
// straight
private double driveStraightConstant = .1;

private double leftChange = 0, rightChange = 0;

private double[] prevEncoderValues =
    {0, 0};

/**
 * Sets how much the robot should correct while driving straight.
 * 
 * @param value
 *            Percentage (0.0 to 1.0)
 */
public void setDriveStraightConstant (double value)
{
    this.driveStraightConstant = value;
}

/**
 * Turns the robot to a certain angle using the robot's turning circle to find
 * the arc-length.
 * 
 * @param angle
 *            How far the robot should turn. Negative angle turns left, positive
 *            turns right. (In Degrees)
 * @param speed
 *            How fast the robot should turn (0 to 1.0)
 * @return Whether or not the robot has finished turning
 */
public boolean turnDegrees (int angle, double speed)
{
    // Only reset the encoders on the method's first start.
    if (turnDegreesInit == true)
        {
        this.resetEncoders();
        turnDegreesInit = false;
        }

    // Tests whether any encoder has driven the arc-length of the angle
    // (angle x radius)// took out +15 on Nov 4

    // Only check 4 encoders if we have a four wheel drive system
    // if two wheel drive checks, one of each side(called rear of that side)
    if (this.getEncoderDistanceAverage(
            WheelGroups.ALL) > Math.toRadians(Math.abs(angle))
                    * TURNING_RADIUS)
        {
        // We have finished turning!
        this.getTransmission().stop();
        turnDegreesInit = true;
        return true;
        }

    // Change which way the robot turns based on whether the angle is
    // positive or negative
    if (angle < 0)
        {
        this.getTransmission().drive(-speed, speed);
        }
    else
        {
        this.getTransmission().drive(speed, -speed);
        }

    return false;
}

/**
 * Turns the robot based on values obtained from a gyroscopic sensor.
 * 
 * @param angle
 *            At what angle we should turn, in degrees. Negative is left,
 *            positive is right.
 * @param speed
 *            How fast we should turn, in decimal percentage (0.0 to 1.0)
 * @return
 *         Whether or not we have finished turning.
 */
public boolean turnDegreesGyro (int angle, double speed)
{
    // Reset the gyro on first start
    if (turnDegreesGyroInit)
        {
        this.gyro.reset();
        turnDegreesGyroInit = false;
        }

    // If we have traveled the number of degrees in any direction, stop.
    if (Math.abs(gyro.getAngle()) > Math.abs(angle))
        {
        this.getTransmission().stop();
        turnDegreesGyroInit = true;
        return true;
        }

    // Turn the robot based on whether we are going left or right.
    if (angle < 0)
        {
        this.getTransmission().drive(-speed, speed);
        }
    else
        {
        this.getTransmission().drive(speed, -speed);
        }

    return false;
}

private boolean turnDegreesGyroInit = true;

// variable to determine if it is the first time running a method
private boolean turnDegreesInit = true;


// ================GAME SPECIFIC FUNCTIONS================
/*
 * Driving functions that change from game to game, such as using the camera
 * to score, etc.
 */

/**
 * Drives using the camera until it hits CAMERA_NO_LONGER_WORKS inches, where it
 * then drives using the ultrasonic
 * 
 * Multiply the compensationFactor by speed to determine what values we are
 * sending to the motor controller
 * 
 * @param compensationFactor
 *            have the compensation factor greater than 1 and less than 1.8
 * @param speed
 *            have the speed greater than 0 and less than 1
 * @return true if the robot has driven all the way to the front of the scale,
 *         and false if it hasn't
 */
public boolean driveToSwitch (double compensationFactor, double speed)
{
    if (this.frontUltrasonic
            .getDistanceFromNearestBumper() > CAMERA_NO_LONGER_WORKS)
        {
        this.getCameraCenterValue();
        if (this.getCameraCenterValue() >= SWITCH_CAMERA_CENTER
                - CAMERA_DEADBAND
                && this.getCameraCenterValue() <= SWITCH_CAMERA_CENTER
                        + CAMERA_DEADBAND)
            {
            vision = visionDriveState.CENTER;
            }
        else if (this.getCameraCenterValue() > SWITCH_CAMERA_CENTER
                + CAMERA_DEADBAND)
            {
            // center is too far right, drive faster on the left
            vision = visionDriveState.TOO_RIGHT;
            }
        else
            {
            // center is too far left, drive faster on the right
            vision = visionDriveState.TOO_LEFT;
            }
        }
    else
        {
        if (this.frontUltrasonic
                .getDistanceFromNearestBumper() <= STOP_ROBOT)
            {
            vision = visionDriveState.STOP;
            this.brake(null);
            }
        vision = visionDriveState.DRIVE_BY_ULTRASONIC;
        }
    switch (vision)
        {
        case TOO_LEFT:
            this.getTransmission().drive(speed * compensationFactor,
                    speed);
            break;
        case TOO_RIGHT:
            this.getTransmission().drive(speed,
                    speed * compensationFactor);
            break;
        case CENTER:
            driveStraight(speed, false);
            break;
        case DRIVE_BY_ULTRASONIC:
            driveStraight(speed, false);
            break;
        case STOP:
            brake(null);
            this.getTransmission().drive(0, 0);
            return true;

        }
    return false;
}


/**
 * Method to test the vision code without the ultrasonic
 * 
 * @param speed
 * @param compensationFactor
 */
public void visionTest (double compensationFactor, double speed)
{
    this.getCameraCenterValue();
    // System.out.println("Center for the vision : " + center);
    if (this.getCameraCenterValue() >= SWITCH_CAMERA_CENTER
            - CAMERA_DEADBAND
            && this.getCameraCenterValue() <= SWITCH_CAMERA_CENTER
                    + CAMERA_DEADBAND)
        {
        // driveStraight(speed, false);
        System.out.println("We are aligned in the center");
        }
    else if (this.getCameraCenterValue() > SWITCH_CAMERA_CENTER
            + CAMERA_DEADBAND)
        {
        // center is too far right, drive faster on the left
        this.getTransmission().drive(speed * compensationFactor,
                speed);
        System.out.println("We're too left");
        }
    else
        {
        // center is too far left, drive faster on the right
        this.getTransmission().drive(speed,
                speed * compensationFactor);
        System.out.println("We're too right");
        }
}

private visionDriveState vision;

private enum visionDriveState
    {
TOO_LEFT, TOO_RIGHT, CENTER, DRIVE_BY_ULTRASONIC, STOP
    }

/**
 * @return the current center value
 */
public double getCameraCenterValue ()
{
    visionProcessor.processImage();
    if (visionProcessor.getParticleReports().length >= 2)
        {
        center = (visionProcessor.getNthSizeBlob(0).center.x
                + visionProcessor.getNthSizeBlob(1).center.x) / 2;
        // System.out.println("TWO BLOBS");
        }
    else if (visionProcessor.getParticleReports().length == 1)
        {
        center = visionProcessor.getNthSizeBlob(0).center.x;
        // System.out.println("ONE BLOBS");
        }
    else
        {
        center = SWITCH_CAMERA_CENTER;
        // System.out.println("NO BLOBS");
        }
    return center;
}

/**
 * Hopefully will align to proper distance w. scale
 * then raise fork lift and eject cube
 * 
 * 
 * param speed
 * Set to negative for too close ajustment
 * 
 * @return true when completed
 * 
 * 
 */

public boolean alignToScale (double speed, double deadband)
{


    // Started align to scale
    // todo optimize deadband to distance

    // checks if in proper distance
    // ROBOT_TO_SCALE_DISTANCE 68-36 =32
    if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE

            && this.rearUltrasonic
                    .getDistanceFromNearestBumper() >= ROBOT_TO_SCALE_DISTANCE
                            - deadband)
        {
        System.out.println("Our distance to the scale is correct");

        // Hardware.tractionDrive.drive(0, 0);
        speed = 0;
        aligned = true;
        // start the move forklift switch

        if (Hardware.cubeManipulator.scoreSwitch())
            {
            return true;
            }



        }
    // if to far from scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE
                    - deadband
    /* && aligned == false */)
        {
        System.out.println("We are too close to the scale");
        Hardware.cubeManipulator.setLiftPosition(0, 0);
        Hardware.transmission.drive(speed, speed);
        }
    // if to close to scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() > ROBOT_TO_SCALE_DISTANCE
    /* && aligned == false */)
        {
        System.out.println("We are to far from the scale");
        Hardware.cubeManipulator.setLiftPosition(0, 0);
        Hardware.transmission.drive(-speed, -speed);
        }
    return false;
}

boolean aligned = false;

// ---------------------------------
// THis is the distance we expect to move
// during a brake call (in inches)
// ---------------------------------
private double distanceRequiredToBrake = 3.0;

// ================VISION TUNABLES================
private final double CAMERA_NO_LONGER_WORKS = 38;
// 24

private final double CAMERA_DEADBAND = 10;

private final double STOP_ROBOT = 20;
// 6

// TODO TEST TO FIND ACTUAL VALUE
private final double SWITCH_CAMERA_CENTER = 115;

// ================VISION VARIABLES================
private double center = 115;

// ================TUNABLES================
//
private static final double SCALE_TO_WALL_DISTANCE = 68;

private static final double ROBOT_LENGTH = 36;// TODO magic number

private static final double ROBOT_TO_SCALE_DISTANCE = SCALE_TO_WALL_DISTANCE
        - ROBOT_LENGTH;

// Number of milliseconds that will pass before collecting data on encoders
// for driveStraight and brake
private static final int COLLECTION_TIME = 100;

// The distance from the left side wheel to the right-side wheel divided by
// 2, in inches. Used in turnDegrees.
// Nov 4 changed from 16 to 17
private static final int TURNING_RADIUS = 11;

private static final int INIT_TIMEOUT = 300;// Milliseconds until the
                                            // initialization should reset.
}
