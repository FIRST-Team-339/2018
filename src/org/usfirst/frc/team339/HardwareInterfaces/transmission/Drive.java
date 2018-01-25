package org.usfirst.frc.team339.HardwareInterfaces.transmission;

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
        UltraSonic rearUltrasonic, KilroyGyro gyro)
{
    this.transmissionType = transmission.getType();
    this.leftRearEncoder = leftEncoder;
    this.rightRearEncoder = rightEncoder;

    this.frontUltrasonic = frontUltrasonic;
    this.rearUltrasonic = rearUltrasonic;
    this.gyro = gyro;

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
                    "There was an error setting up the DRIVE class. Please check the declaration for a valid transmission object.");
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
 * All wheels combined
 */
ALL,
/**
 * Both front and back on the left side
 */
LEFT_SIDE,
/**
 * Both front and back on the right side
 */
RIGHT_SIDE,
/**
 * Both left and right rear wheels
 */
REAR
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
            if (transmissionType == TransmissionType.MECANUM
                    || transmissionType == TransmissionType.TANK)
                {
                leftFrontEncoder.setDistancePerPulse(value);
                rightFrontEncoder.setDistancePerPulse(value);
                leftRearEncoder.setDistancePerPulse(value);
                rightRearEncoder.setDistancePerPulse(value);
                }
            else if (transmissionType == TransmissionType.TRACTION)
                {
                leftRearEncoder.setDistancePerPulse(value);
                rightRearEncoder.setDistancePerPulse(value);
                }
            break;
        case LEFT_FRONT:
            leftFrontEncoder.setDistancePerPulse(value);
            break;
        case RIGHT_FRONT:
            rightFrontEncoder.setDistancePerPulse(value);
            break;
        case LEFT_REAR:
            leftRearEncoder.setDistancePerPulse(value);
            break;
        case RIGHT_REAR:
            rightRearEncoder.setDistancePerPulse(value);
            break;
        default:
            break;
        }
}

/**
 * Sets the encoder's stored pulses back to zero.
 */
public void resetEncoders ()
{
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        {
        leftFrontEncoder.reset();
        rightFrontEncoder.reset();
        leftRearEncoder.reset();
        rightRearEncoder.reset();
        }
    else if (transmissionType == TransmissionType.TRACTION)
        {
        leftRearEncoder.reset();
        rightRearEncoder.reset();
        }
}

/**
 * Gets the averages of certain wheel groups. All values are the absolute value
 * to stop
 * negative numbers from affecting the average.
 * 
 * @param encoderGroup
 * @return
 */
public double getEncoderDistanceAverage (WheelGroups encoderGroup)
{
    switch (encoderGroup)
        {
        case ALL:
            return (Math.abs(leftFrontEncoder.getDistance())
                    + Math.abs(rightFrontEncoder.getDistance())
                    + Math.abs(leftRearEncoder.getDistance())
                    + Math.abs(rightRearEncoder.getDistance())) / 4.0;
        case LEFT_SIDE:
            return (Math.abs(leftFrontEncoder.getDistance())
                    + Math.abs(leftRearEncoder.getDistance())) / 2.0;
        case RIGHT_SIDE:
            return (Math.abs(rightFrontEncoder.getDistance())
                    + Math.abs(rightRearEncoder.getDistance())) / 2.0;
        case REAR:
            return (Math.abs(leftRearEncoder.getDistance())
                    + Math.abs(rightRearEncoder.getDistance())) / 2.0;
        default:
            return 0.0;
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
    if (transmissionType == TransmissionType.MECANUM
            || transmissionType == TransmissionType.TANK)
        return (Math.abs(leftFrontEncoder.getDistance()) > length
                || Math.abs(rightFrontEncoder.getDistance()) > length
                || Math.abs(leftRearEncoder.getDistance()) > length
                || Math.abs(rightRearEncoder.getDistance()) > length);
    return (Math.abs(leftRearEncoder.getDistance()) > length
            || Math.abs(rightRearEncoder.getDistance()) > length);

}

// ================ DRIVE METHODS ================

/**
 * Resets the Drive class's functions, in case they were cut short.
 */
public void reset ()
{
    this.brakeInit = true;
    this.driveInchesInit = true;
    this.driveStraightInchesInit = true;
    this.turnDegreesInit = true;
}

/**
 * Stops the robot suddenly, to prevent drifting during autonomous functions,
 * and increase the precision.
 * 
 * @return
 *         Whether or not the robot has stopped moving.
 */
public boolean brake ()
{
    // Reset the encoders on the first startup
    if (brakeInit)
        {
        this.resetEncoders();
        this.brakeMotorPower = new double[]
            {0, 0, 0, 0};
        brakeInit = false;
        return false;
        }
    // Use a proportional loop to set the powers of each motor
    this.brakeMotorPower[0] = inRange(
            leftRearEncoder.getRate() * brakeScalar);
    this.brakeMotorPower[1] = inRange(
            rightRearEncoder.getRate() * brakeScalar);

    getTransmission().getSpeedController(MotorPosition.LEFT_REAR)
            .set(-this.brakeMotorPower[0]);
    getTransmission().getSpeedController(MotorPosition.RIGHT_REAR)
            .set(-this.brakeMotorPower[1]);

    // Only test the other encoders if it is a 4 wheel system
    if (transmissionType != TransmissionType.TRACTION)
        {
        this.brakeMotorPower[2] = inRange(leftFrontEncoder.getRate()
                * brakeScalar);
        this.brakeMotorPower[3] = inRange(rightFrontEncoder.getRate()
                * brakeScalar);
        getTransmission().getSpeedController(MotorPosition.LEFT_FRONT)
                .set(-this.brakeMotorPower[2]);
        getTransmission().getSpeedController(MotorPosition.RIGHT_FRONT)
                .set(-this.brakeMotorPower[3]);
        }

    // Use boolean short circuiting to test the powers and transmission types
    if (brakeMotorPower[0] < BRAKE_DEADBAND
            && brakeMotorPower[1] < BRAKE_DEADBAND
            && (transmissionType == TransmissionType.TRACTION
                    || (brakeMotorPower[2] < BRAKE_DEADBAND
                            && brakeMotorPower[3] < BRAKE_DEADBAND)))
        {
        this.getTransmission().stop();
        this.brakeInit = true;
        return true;
        }

    return false;
}

private boolean brakeInit = true;

private long previousBrakeTime = 1;

private double[] brakeMotorPower =
    {0, 0, 0, 0};

private double[] prevBrakeVals =
    {0, 0, 0, 0};

private double brakeScalar = 1;

/**
 * Sets the constant that is multiplied by delta-encoder values when braking.
 * 
 * @param brakeScalar
 *            The value to be set. Must be higher than 0.
 */
public void setBrakeScalingFactor (double brakeScalar)
{
    this.brakeScalar = brakeScalar;
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

    this.getTransmission().driveRaw(speed, speed);
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

    // Check encoders to see if the distance has been driven
    if (this.transmissionType == TransmissionType.MECANUM
            || this.transmissionType == TransmissionType.TANK)
        {
        // Check all encoders if it is a four wheel drive system.
        if (this.getEncoderDistanceAverage(WheelGroups.ALL) > distance)
            {
            this.getTransmission().stop();
            driveStraightInchesInit = true;
            return true;
            }
        }
    else
        {
        // Only check the rear encoders if it is a two wheel drive system.
        if (this.getEncoderDistanceAverage(WheelGroups.REAR) > distance)
            {
            this.getTransmission().stop();
            driveStraightInchesInit = true;
            return true;
            }
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
    this.mecanumTransmission.driveRaw(speed,
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
    if (System.currentTimeMillis() - lastAccelerateTime > accelTimeout)
        {
        lastAccelerateTime = System.currentTimeMillis();
        accelMotorPower = 0;
        }

    // main acceleration maths
    double deltaSeconds = (System.currentTimeMillis()
            - lastAccelerateTime) / 1000.0;
    accelMotorPower += deltaSeconds / time;

    //Drive the robot based on the times and speeds
    this.getTransmission().driveRaw(
            leftSpeed * inRange(accelMotorPower),
            rightSpeed * inRange(accelMotorPower));

    //Reset the "timer"
    lastAccelerateTime = System.currentTimeMillis();
    
    if (accelMotorPower > 1.0)
        return true;
    // We are not done accelerating
    return false;
}

private double accelMotorPower = 0;// Power sent to each motor: [Left, Right]

private long lastAccelerateTime = 0; // Milliseconds

private double defaultAcceleration = 1;//Seconds

private int accelTimeout = 300;// Milliseconds

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
 */
public void driveStraight (double speed, boolean accelerate)
{
    // Only check encoders if the right amount of time has elapsed
    // (collectionTime).
    if (System.currentTimeMillis() > driveStraightOldTime
            + COLLECTION_TIME)
        {
        // Reset the "timer"
        driveStraightOldTime = System.currentTimeMillis();
        // Only use the four encoders if the robot uses a four-wheel system
        if (transmissionType == TransmissionType.MECANUM
                || transmissionType == TransmissionType.TANK)
            {
            // Calculate how much has changed between the last collection
            // time and now
            leftChange = (leftFrontEncoder.getDistance()
                    + leftRearEncoder.getDistance()) - prevEncoderValues[0];
            rightChange = (rightFrontEncoder.getDistance()
                    + rightRearEncoder.getDistance()) - prevEncoderValues[1];
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
            leftChange = leftRearEncoder.getDistance() - prevEncoderValues[0];
            rightChange = rightRearEncoder.getDistance() - prevEncoderValues[1];
            // Setup the previous values for the next collection run
            prevEncoderValues[0] = leftRearEncoder.getDistance();
            prevEncoderValues[1] = rightRearEncoder.getDistance();
            }
        }
    double leftMotorVal = speed + (.15 * inRange(rightChange - leftChange));
    double rightMotorVal = speed + (.15 * inRange(leftChange - rightChange));
    // Changes how much the robot corrects by how off course it is. The
    // more off course, the more it will attempt to correct.
    if (accelerate == false)
        this.getTransmission().driveRaw(leftMotorVal,rightMotorVal);
    else
        this.accelerateTo(leftMotorVal, rightMotorVal, defaultAcceleration);

}

private double leftChange = 1, rightChange = 1;

private double [] prevEncoderValues =
    {100, 100};
// Preset to 1 to avoid divide by zero errors.

// Used for calculating how much time has passed for driveStraight
private long driveStraightOldTime = 0;

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
    if (this.transmissionType == TransmissionType.MECANUM
            || this.transmissionType == TransmissionType.TANK)
        {
        // Only check 4 encoders if we have a four wheel drive system
        if (this.getEncoderDistanceAverage(
                WheelGroups.ALL) > Math.toRadians(Math.abs(angle))
                        * TURNING_RADIUS)
            {
            // We have finished turning!
            this.getTransmission().stop();
            turnDegreesInit = true;
            return true;
            }
        }
    else
        {
        // Only check 2 encoders if we have a two wheel drive system
        if (this.getEncoderDistanceAverage(
                WheelGroups.REAR) > Math.toRadians(Math.abs(angle))
                        * TURNING_RADIUS)
            {
            // We have finished turning!
            this.getTransmission().stop();
            turnDegreesInit = true;
            return true;
            }
        }

    // Change which way the robot turns based on whether the angle is
    // positive or negative
    if (angle < 0)
        {
        this.getTransmission().driveRaw(-speed, speed);
        }
    else
        {
        this.getTransmission().driveRaw(speed, -speed);
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
        this.getTransmission().driveRaw(-speed, speed);
        }
    else
        {
        this.getTransmission().driveRaw(speed, -speed);
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
        visionProcessor.processImage();
        double center = (visionProcessor.getNthSizeBlob(0).center.x
                + visionProcessor.getNthSizeBlob(1).center.x) / 2;

        if (center >= SWITCH_CAMERA_CENTER - CAMERA_DEADBAND
                && center <= SWITCH_CAMERA_CENTER + CAMERA_DEADBAND)
            {
            driveStraight(speed, false);
            }
        else if (center > SWITCH_CAMERA_CENTER + CAMERA_DEADBAND)
            {
            // center is too far right, drive faster on the left
            this.getTransmission().driveRaw(speed * compensationFactor,
                    speed);
            }
        else
            {
            // center is too far left, drive faster on the right
            this.getTransmission().driveRaw(speed,
                    speed * compensationFactor);
            }
        }
    else
        {
        if (this.frontUltrasonic
                .getDistanceFromNearestBumper() <= STOP_ROBOT)
            {
            this.brake();
            }
        this.driveStraight(speed, false);
        return true;
        }
    return false;
}

// ================VISION TUNABLES================
private final double CAMERA_NO_LONGER_WORKS = 24;

private final double CAMERA_DEADBAND = 2;

private final double STOP_ROBOT = 6;

// TODO TEST TO FIND ACTUAL VALUE
private final double SWITCH_CAMERA_CENTER = 45;

// ================TUNABLES================

// Number of milliseconds that will pass before collecting data on encoders
// for driveStraight and brake
private static final int COLLECTION_TIME = 100;

// The change in inches per [COLLECTION_TIME] where the robot is considered
// "stopped".
private static final double BRAKE_DEADBAND = .2;

// The distance from the left side wheel to the right-side wheel divided by
// 2, in inches. Used in turnDegrees.
// Nov 4 changed from 16 to 17
private static final int TURNING_RADIUS = 16;
}
