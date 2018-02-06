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

/**
 * Contains all game specific vision code
 * 
 * @author Becky Button
 */
public class DriveWithCamera extends Drive
{

private UltraSonic frontUltrasonic = null;

private VisionProcessor visionProcessor = null;

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
        KilroyGyro gyro, VisionProcessor visionProcessor)
{
    super(transmission, leftEncoder, rightEncoder, frontUltrasonic,
            rearUltrasonic, gyro, visionProcessor);

    this.frontUltrasonic = frontUltrasonic;
    this.visionProcessor = visionProcessor;
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
public boolean driveToSwitch (double compensationFactor, double speed)
{
    // if the robot can still see the vision targets, drive by camera
    if (this.frontUltrasonic
            .getDistanceFromNearestBumper() > CAMERA_NO_LONGER_WORKS)
        {

        // gets the position of the center
        this.getCameraCenterValue();
        this.visionProcessor.setRelayValue(Value.kForward);

        // if we are aligned the center, we will drive straight
        if (this.getCameraCenterValue() >= SWITCH_CAMERA_CENTER
                - CAMERA_DEADBAND
                && this.getCameraCenterValue() <= SWITCH_CAMERA_CENTER
                        + CAMERA_DEADBAND)
            {
            driveStraight(speed, false);
            }
        // if the center is to the right of our center set by the
        // SWITCH_CAMERA_CENTER, correct by driving faster on the left
        else if (this.getCameraCenterValue() > SWITCH_CAMERA_CENTER
                + CAMERA_DEADBAND)
            {
            // center is too far right, drive faster on the left
            this.getTransmission().drive(speed,
                    speed * compensationFactor);
            }
        // if the center is to the left of our center set by the
        // SWITCH_CAMERA_CENTER, correct by driving faster on the right
        else
            {
            // center is too far left, drive faster on the right
            this.getTransmission().drive(speed * compensationFactor,
                    speed);
            }
        }
    // if we can no longer see vision targets, drive by ultrasonic, turn off the
    // relay
    else
        {
        this.visionProcessor.setRelayValue(Value.kReverse);

        // if we are too close to the wall, brake, then set all motors to zero,
        // else drive by ultrasonic
        if (this.frontUltrasonic
                .getDistanceFromNearestBumper() <= STOP_ROBOT)
            {
            this.brake(BrakeType.AFTER_DRIVE);
            this.getTransmission().drive(0, 0);
            return true;
            }
        driveStraight(speed, false);
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

/**
 * Gets the center x value of of the vision targets
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
private final double CAMERA_NO_LONGER_WORKS = 38;
// 24

// the number in pixels that the center we are looking for can be off
private final double CAMERA_DEADBAND = 10;

// the distance in inches in which we stop the robot
private final double STOP_ROBOT = 20;
// 6

// TODO This is for nessie, test for the new robot
private final double SWITCH_CAMERA_CENTER = 115;

// ================VISION TUNABLES================
private double center = 115;

}