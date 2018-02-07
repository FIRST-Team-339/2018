package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;

public class ScaleAlignment
{

private UltraSonic rearUltrasonic = null;



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
public ScaleAlignment (UltraSonic ultrasonic)
{


    this.rearUltrasonic = ultrasonic;

}

/**
 * Score cube on to scale
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

    // TODO optimize deadband to distance

    // checks if in proper distance
    // ROBOT_TO_SCALE_DISTANCE 72-36 =36
    if (Hardware.rearUltraSonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE
            && Hardware.rearUltraSonic
                    .getDistanceFromNearestBumper() >= ROBOT_TO_SCALE_DISTANCE
                            - deadband)
        {
        System.out.println("Our distance to the scale is correct");
        aligned = true;
        speed = 0;
        Hardware.transmission.drive(0, 0);

        return true;
        }
    // if to far from scale
    else if (Hardware.rearUltraSonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE
                    - deadband
            && aligned == false)
        {
        System.out.println("We are too close to the scale");
        Hardware.cubeManipulator.setLiftPosition(0, 0);
        Hardware.transmission.drive(speed, speed);
        }
    // if to close to scale
    else if (Hardware.rearUltraSonic
            .getDistanceFromNearestBumper() > ROBOT_TO_SCALE_DISTANCE
            && aligned == false)
        {
        System.out.println("We are to far from the scale");
        Hardware.cubeManipulator.setLiftPosition(0, 0);
        Hardware.transmission.drive(-speed, -speed);
        }
    return false;
}

boolean aligned = false;


private static final double SCALE_TO_WALL_DISTANCE = 68;

private static final double ROBOT_LENGTH = 36;// TODO magic number

private static final double ROBOT_TO_SCALE_DISTANCE = SCALE_TO_WALL_DISTANCE
        - ROBOT_LENGTH;

}
