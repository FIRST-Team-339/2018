package org.usfirst.frc.team339.HardwareInterfaces;

import org.usfirst.frc.team339.HardwareInterfaces.transmission.Drive;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.MecanumTransmission;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TankTransmission;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TractionTransmission;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.TransmissionType;
import org.usfirst.frc.team339.vision.VisionProcessor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Contains all game specific vision code, including code to drive to the switch
 * using vision
 * 
 * @author: Becky Button
 */
public class DriveWithCamera extends Drive
{

private TankTransmission tankTransmission = null;

private TractionTransmission tractionTransmission = null;

private MecanumTransmission mecanumTransmission = null;

private Encoder leftFrontEncoder = null, rightFrontEncoder = null,
        leftRearEncoder = null, rightRearEncoder = null;

private UltraSonic frontUltrasonic = null;

private UltraSonic rearUltrasonic = null;

private Gyro gyro = null;

private VisionProcessor visionProcessor = null;

private final TransmissionType transmissionType;

/**
 * Creates the drive with camera object. If a sensor listed is not used (except
 * for
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
public DriveWithCamera (TransmissionBase transmission,
        Encoder leftFrontEncoder,
        Encoder rightFrontEncoder,
        Encoder leftRearEncoder, Encoder rightRearEncoder,
        Gyro gyro,
        VisionProcessor visionProcessor)
{
    super(transmission, leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder,
            gyro);

    this.visionProcessor = visionProcessor;
    this.transmissionType = transmission.getType();
    this.leftFrontEncoder = leftFrontEncoder;
    this.rightFrontEncoder = rightFrontEncoder;
    this.leftRearEncoder = leftRearEncoder;
    this.rightRearEncoder = rightRearEncoder;
    this.gyro = gyro;

}

/**
 * Creates drive with camera object
 * 
 * @param transmission
 *            The robot's transmission object
 * @param leftEncoder
 *            The left encoder
 * @param rightEncoder
 *            The right encoder
 * @param frontUltrasonic
 *            The robot's front ultrasonic
 * @param rearUltrasonic
 *            The robots's read ultrasonic
 * @param gyro
 *            A sensor that uses a spinning disk to measure rotation.
 * @param visionProcessor
 *            The camera's vision processing code, as a sensor.
 * 
 */
public DriveWithCamera (TransmissionBase transmission,
        Encoder leftEncoder, Encoder rightEncoder,
        UltraSonic frontUltrasonic, UltraSonic rearUltrasonic,
        Gyro gyro, VisionProcessor visionProcessor)
{
    super(transmission, leftEncoder, rightEncoder, gyro);

    this.frontUltrasonic = frontUltrasonic;
    this.rearUltrasonic = rearUltrasonic;
    this.visionProcessor = visionProcessor;
    this.transmissionType = transmission.getType();
    this.leftRearEncoder = leftEncoder;
    this.rightRearEncoder = rightEncoder;
    this.gyro = gyro;
}



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
public boolean driveToSwitch (double speed)
{
    // if the robot can still see the vision targets, drive by camera
    if (this.frontUltrasonic
            .getDistanceFromNearestBumper() > CAMERA_NO_LONGER_WORKS)
        {
        state = driveWithCameraState.DRIVE_WITH_CAMERA;
        }

    // if we can no longer see vision targets, drive by ultrasonic, turn off the
    // relay
    else
        {
        // turn off the ring light
        this.visionProcessor.setRelayValue(Value.kReverse);

        if (this.frontUltrasonic
                .getDistanceFromNearestBumper() <= DISTANCE_FROM_WALL_TO_STOP)
            {
            state = driveWithCameraState.STOP;
            }
        else
            {
            state = driveWithCameraState.DRIVE_WITH_US;
            }
        }
    switch (state)
        {
        case DRIVE_WITH_CAMERA:
            // gets the position of the center
            double center = this.getCameraCenterValue();
            // turns on the ring light
            this.visionProcessor.setRelayValue(Value.kForward);
            // if we are aligned the center, we will drive straight
            // if (center >= SWITCH_CAMERA_CENTER
            // - CAMERA_DEADBAND
            // && center <= SWITCH_CAMERA_CENTER
            // + CAMERA_DEADBAND)
            // {
            // driveStraight(speed, false);
            // System.out.println("DRIVE STRAIGHT");
            // }
            // // if the switch center is to the right of our center set by the
            // // SWITCH_CAMERA_CENTER, correct by driving faster on the left
            // else
            if (center >= SWITCH_CAMERA_CENTER)
                {
                // the switch's center is too far right, drive faster on the
                // left
                // System.out.println("WE ARE TOO RIGHT");
                this.getTransmission().drive(speed + DRIVE_CORRECTION,
                        speed - DRIVE_CORRECTION);
                }
            // if the switch center is to the left of our center set by the
            // SWITCH_CAMERA_CENTER, correct by driving faster on the right
            else
                {
                // the switch's center is too far left, drive faster on the
                // right
                // System.out.println("WE ARE TOO LEFT");
                this.getTransmission().drive(speed - DRIVE_CORRECTION,
                        speed + DRIVE_CORRECTION);
                }
            break;
        case STOP:
            // if we are too close to the wall, brake, then set all motors to
            // zero, else drive by ultrasonic
            this.getTransmission().drive(0, 0);
            return true;
        case DRIVE_WITH_US:
            driveStraight(speed, false);
            break;
        default:
            this.getTransmission().drive(0, 0);
            break;
        }
    return false;
}

private driveWithCameraState state;

private enum driveWithCameraState
    {
DRIVE_WITH_CAMERA, DRIVE_WITH_US, STOP
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
    System.out.println("Center for the vision : " + center);
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
        // center is too far left, drive faster on the right
        this.getTransmission().drive(speed,
                speed * compensationFactor);
        System.out.println("We're too right");
        }
    else
        {
        // center is too far right, drive faster on the left
        this.getTransmission().drive(speed * compensationFactor,
                speed);
        System.out.println("We're too left");
        }
}

/**
 * Gets the center x value of of the vision targets (average of the x values
 * of both visions targets)
 * 
 * @return the current center x value
 */
public double getCameraCenterValue ()
{
    visionProcessor.processImage();
    // if we have at least two blobs, the center is equal to the average center
    // x position of the 1st and second largest blobs
    if (visionProcessor.getParticleReports().length >= 2)
        {
        center = (visionProcessor.getNthSizeBlob(0).center.x
                + visionProcessor.getNthSizeBlob(1).center.x) / 2;
        // System.out.println("TWO BLOBS");
        }
    // if we only can detect one blob, the center is equal to the center x
    // position of the blob
    else if (visionProcessor.getParticleReports().length == 1)
        {
        center = visionProcessor.getNthSizeBlob(0).center.x;
        // System.out.println("ONE BLOBS");
        }
    // if we don't have any blobs, set the center equal to the constanct center,
    // we can use this to just drive straight
    else
        {
        center = SWITCH_CAMERA_CENTER;
        // System.out.println("NO BLOBS");
        }
    return center;
}

// ================VISION CONSTANTS================
// the distance in inches in which we drive the robot straight using the
// ultrasonic
private final double CAMERA_NO_LONGER_WORKS = 38 + 50;
// 24

// the number in pixels that the center we are looking for can be off
private final double CAMERA_DEADBAND = 7;

// the distance from the wall (in inches) where we start stopping the robot
private final double DISTANCE_FROM_WALL_TO_STOP = 20 + 50;
// 6

// TODO This is for nessie, test for the new robot
private final double SWITCH_CAMERA_CENTER = 160;
// 160;

private final double DRIVE_CORRECTION = .4;

// ================VISION TUNABLES================
// Gets the center x value of the vision targets (average of the x values
// of both vision targets, or of the only one if there is only one blob: see
// getCameraCenterValue())
private double center = 115;

}
