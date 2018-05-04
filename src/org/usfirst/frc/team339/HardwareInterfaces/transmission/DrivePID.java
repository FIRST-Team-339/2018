package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.Utils.KilroyPID;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An extension of the Drive class that overrides the methods to use PID loops
 * instead of static-correcting methods.
 * 
 * @author Ryan McGee
 *
 */
public class DrivePID extends Drive
{

private final KilroyPID[] motorPID;

/**
 * Create the DrivePID Object with a 4 encoder system
 * 
 * @param transmission
 * @param leftFrontEncoder
 * @param rightFrontEncoder
 * @param leftRearEncoder
 * @param rightRearEncoder
 * @param gyro
 */
public DrivePID (TransmissionBase transmission,
        Encoder leftFrontEncoder, Encoder rightFrontEncoder,
        Encoder leftRearEncoder, Encoder rightRearEncoder,
        GyroBase gyro)
{
    // Create the Drive class this is extending, as to override the methods
    // to use PID instead of static constants.
    super(transmission, leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder, gyro);
    motorPID = new KilroyPID[4];
    motorPID[0] = new KilroyPID(
            transmission.getSpeedController(MotorPosition.LEFT_REAR),
            leftRearEncoder);
    motorPID[1] = new KilroyPID(
            transmission.getSpeedController(MotorPosition.RIGHT_REAR),
            rightRearEncoder);
    motorPID[2] = new KilroyPID(
            transmission.getSpeedController(MotorPosition.LEFT_FRONT),
            leftFrontEncoder);
    motorPID[3] = new KilroyPID(
            transmission.getSpeedController(MotorPosition.RIGHT_FRONT),
            rightFrontEncoder);
    // TODO Auto-generated constructor stub
}

/**
 * Create the DrivePID Object with a 2 encoder system
 * 
 * @param transmission
 * @param leftEncoder
 * @param rightEncoder
 * @param gyro
 */
public DrivePID (TransmissionBase transmission, Encoder leftEncoder,
        Encoder rightEncoder, GyroBase gyro)
{
    super(transmission, leftEncoder, rightEncoder, gyro);
    motorPID = new KilroyPID[2];
    motorPID[0] = new KilroyPID(
            transmission.getSpeedController(MotorPosition.LEFT_REAR),
            leftEncoder);
    motorPID[1] = new KilroyPID(
            transmission.getSpeedController(MotorPosition.RIGHT_REAR),
            rightEncoder);
}

/**
 * Resets the all drive function's initialization.
 */
public void reset ()
{
    super.reset();
    this.turnDegreesInit = true;
    for (KilroyPID pid : motorPID)
        pid.setEnabled(false);
}

/**
 * Turns the robot X degrees on the spot, using left and right encoders.
 * 
 * @param degrees
 *            How far the robot should turn, in degrees. Positive for clockwise,
 *            negative for counterclockwise.
 * 
 * @param speed
 *            How fast the robot should turn, in positive percentage.
 * 
 * @return whether or not the robot has finished turning.
 */
public boolean turnDegrees (int degrees, double speed)
{
    // Reset the PID loop and set the PID values if first start
    if (turnDegreesInit == true)
        {
        // Reset the sensors
        super.resetEncoders();
        for (KilroyPID pid : motorPID)
            {
            // Set PID settings
            pid.resetPID();
            pid.setPIDF(turnPIDTolerance[0], turnPIDTolerance[1],
                    turnPIDTolerance[2], 0);
            pid.setTolerance(turnPIDTolerance[3]);
            }
        // Set the setpoint to each motor. If it's even (0 or 2), then it's the
        // left side of the robot. Odds (1 or 3) are right.
        for (int i = 0; i < motorPID.length; i++)
            {
            if (i % 2 == 0)
                motorPID[i].setSetpoint(
                        degreesToEncoderInches(degrees, false));
            else
                motorPID[i].setSetpoint(
                        -degreesToEncoderInches(degrees, false));
            // Left or right, set them to enabled.
            motorPID[i].setSpeed(speed);
            motorPID[i].setEnabled(true);
            }
        turnDegreesInit = false;
        }

    // If ANY motor is not on target, then keep running.
    boolean isOnTarget = true;
    for (KilroyPID pid : motorPID)
        if (pid.isOnTarget() == false)
            {
            isOnTarget = false;
            break;
            }

    if (isOnTarget == true)
        {
        // We have reached the destination.
        for (KilroyPID pid : motorPID)
            pid.setEnabled(false);

        turnDegreesInit = true;
        return true;
        }
    return false;
}

/**
 * Sets what the PID loops will be looking at while running
 * 
 * @param sourceType
 *            Use kDisplacement for absolute positioning (i.e. turning to a
 *            degree), Use kRate for speed adjustments (i.e. controlling RPM)
 */
private void setAllPIDSourceTypes (PIDSourceType sourceType)
{
    // Rear Encoders or both encoders if 2 wheel drive
    if (getEncoder(MotorPosition.LEFT_REAR) != null)
        getEncoder(MotorPosition.LEFT_REAR)
                .setPIDSourceType(sourceType);
    if (getEncoder(MotorPosition.RIGHT_REAR) != null)
        getEncoder(MotorPosition.RIGHT_REAR)
                .setPIDSourceType(sourceType);
    // Front encoders, if they exist
    if (getEncoder(MotorPosition.LEFT_FRONT) != null)
        getEncoder(MotorPosition.LEFT_FRONT)
                .setPIDSourceType(sourceType);
    if (getEncoder(MotorPosition.RIGHT_FRONT) != null)
        getEncoder(MotorPosition.RIGHT_FRONT)
                .setPIDSourceType(sourceType);
}

// ======================PID Tuning=====================

/**
 * Tunes the PID loop for turning via encoders.
 * 
 * P - Proportional function: Value adds P percent per inch the encoder is off
 * currently. I - Integral function: Value adds I percent per inch the encoder
 * has been off cumulatively D - Derivative function: Value adds D percent per
 * inch per second the error is changing in relation to the setpoint (slope of
 * current position vs error get smaller as the controller slows down to reach
 * it's target)
 * 
 * A high P value will give you a more aggressive correction, but may induce
 * oscillation: Try to avoid that... A high I value will correct any long term
 * "drifting to a side" problems, but again may induce oscillation. A high D
 * value will give you a longer drawn out deceleration curve, and may fix
 * oscillation, but will make the robot take longer reach it's setpoint.
 * 
 * A low tolerance will result in more accurate turns, but may induce a little
 * "wiggle" at the end (which is not bad, but takes more time.)
 * 
 * The point of using a PID loop for turning is to increase the speed of the
 * turn and reduce overshoot / undershoot.
 */
public void tunePID (PIDDriveFunction type)
{
    switch (type)
        {
        case TURN:
            if (tuneTurnDegreesPIDInit == true)
                {
                SmartDashboard.putNumber("Turn P", turnPIDTolerance[0]);
                SmartDashboard.putNumber("Turn I", turnPIDTolerance[1]);
                SmartDashboard.putNumber("Turn D", turnPIDTolerance[2]);
                SmartDashboard.putNumber("Turn Tolerance",
                        turnPIDTolerance[3]);
                tuneTurnDegreesPIDInit = false;
                }

            this.turnPIDTolerance[0] = SmartDashboard
                    .getNumber("Turn P", turnPIDTolerance[0]);
            this.turnPIDTolerance[1] = SmartDashboard
                    .getNumber("Turn I", turnPIDTolerance[1]);
            this.turnPIDTolerance[2] = SmartDashboard
                    .getNumber("Turn D", turnPIDTolerance[2]);
            this.turnPIDTolerance[3] = SmartDashboard
                    .getNumber("Turn Tolerance", turnPIDTolerance[3]);
            return;
        case DRIVESTRAIGHT:
            if (tuneDriveStraightPIDInit == true)
                {
                SmartDashboard.putNumber("Drive Straight P",
                        driveStraightPIDTolerance[0]);
                SmartDashboard.putNumber("Drive Straight I",
                        driveStraightPIDTolerance[1]);
                SmartDashboard.putNumber("Drive Straight D",
                        driveStraightPIDTolerance[2]);
                SmartDashboard.putNumber("Drive Straight Tolerance",
                        driveStraightPIDTolerance[3]);
                tuneTurnDegreesPIDInit = false;
                }

            this.driveStraightPIDTolerance[0] = SmartDashboard
                    .getNumber("Drive Straight P",
                            driveStraightPIDTolerance[0]);
            this.driveStraightPIDTolerance[1] = SmartDashboard
                    .getNumber("Drive Straight I",
                            driveStraightPIDTolerance[1]);
            this.driveStraightPIDTolerance[2] = SmartDashboard
                    .getNumber("Drive Straight D",
                            driveStraightPIDTolerance[2]);
            this.driveStraightPIDTolerance[3] = SmartDashboard
                    .getNumber("Drive Straight Tolerance",
                            driveStraightPIDTolerance[3]);
            return;
        }

}

public enum PIDDriveFunction
    {
TURN, DRIVESTRAIGHT
    }

// ======================Variables======================

private boolean tuneTurnDegreesPIDInit = true;

private double[] turnPIDTolerance =
        // {P, I, D, Tolerance}
        {0, 0, 0, 0};

private boolean turnDegreesInit = true;

private boolean tuneDriveStraightPIDInit = true;

private double[] driveStraightPIDTolerance =
        // {P, I, D, Tolerance}
        {0, 0, 0, 0};
}
