package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;

public class ScaleAlignment
{

private UltraSonic rearUltrasonic = null;



/**
 * Creates AlignToScale object
 *
 * @param ultrasonic
 *            The sensor that finds distance using sound
 */
public ScaleAlignment (UltraSonic ultrasonic)
{


    this.rearUltrasonic = ultrasonic;

}

/**
 * ALign to proper distance with the scale
 * 
 * @return true when completed
 * 
 * @param speed
 *            Do not set negative(0.0 to 1.0)
 * 
 * @param deadband
 * 
 */

public boolean alignToScale (double speed, double deadband)
{
    // Started align to scale
    // todo optimize deadband to distance

    // checks if in proper distance

    // ROBOT_TO_SCALE_DISTANCE 72-36 =36
    if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE

            && this.rearUltrasonic
                    .getDistanceFromNearestBumper() >= ROBOT_TO_SCALE_DISTANCE
                            - deadband)
        {
        System.out.println("Our distance to the scale is correct");
        aligned = true;
        speed = 0;
        System.out.println("Speed: " + speed);
        Hardware.leftDriveMotor.set(0);
        Hardware.rightDriveMotor.set(0);
        // start the move forklift switch

        if (Hardware.cubeManipulator.scoreScale())
            {
            return true;
            }
        System.out.println("Hi");
        Hardware.cubeManipulator.scoreScale();
        }
    // if to far from scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE
                    - deadband)
        {
        System.out.println("Speed: " + speed);
        System.out.println("We are too far from the scale");

        Hardware.transmission.drive(speed, speed);
        }
    // if to close to scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() > ROBOT_TO_SCALE_DISTANCE)
        {
        System.out.println("Speed: " + speed);
        System.out.println("We are to close to the scale");

        Hardware.transmission.drive(-speed, -speed);
        }
    return false;
}

private static boolean aligned = false;

private static final double SCALE_TO_WALL_DISTANCE = 68;

private static final double ROBOT_LENGTH = 36;// TODO 38 on the 2018 robot

private static final double ROBOT_TO_SCALE_DISTANCE = SCALE_TO_WALL_DISTANCE
        - ROBOT_LENGTH;

}
