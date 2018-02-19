package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScaleAlignment
{

private UltraSonic rearUltrasonic = null;

/**
 * Creates AlignToScale object
 *
 * @param ultrasonic
 *            The sensor that finds distance using sound
 * 
 * @author Conner McKevitt
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
 * @author Conner McKevitt
 */

public boolean alignToScale (double speed, double deadband,
        boolean override)
{
    // Started align to scale
    // todo optimize deadband to distance

    // checks if in proper distance

    // ROBOT_TO_SCALE_DISTANCE 72-36 =36
    if (override == true)
        {
        alignOverride = true;
        }
    if (alignOverride == true)
        {
        Hardware.transmission
                .setForTeleop(Robot.KILROY_XIX_GEAR_2_SPEED);
        aligned = false;
        return false;
        }
    if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE
                    - deadband

            && this.rearUltrasonic
                    .getDistanceFromNearestBumper() >= ROBOT_TO_SCALE_DISTANCE
                            - (deadband * 2)
            && alignOverride == false)
        {
        RelativeScale = "correct";
        aligned = true;
        speed = 0;
        System.out.println("Speed: " + speed);
        Hardware.leftDriveMotor.set(0);
        Hardware.rightDriveMotor.set(0);

        // start the move forklift switch
        if (Hardware.cubeManipulator.scoreScale())
            {
            aligned = false;
            return true;
            }

        Hardware.cubeManipulator.scoreScale();
        }
    // if to far from scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE
                    - (deadband * 2)
            && aligned == false && alignOverride == false)
        {
        SmartDashboard.putNumber("Speed", speed);
        RelativeScale = "Too Far";

        Hardware.transmission.drive(speed, speed);
        }
    // if to close to scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() > ROBOT_TO_SCALE_DISTANCE
                    - deadband
            && aligned == false && alignOverride == false)
        {
        SmartDashboard.putNumber("Speed", speed);

        RelativeScale = "Too close";

        Hardware.transmission.drive(-speed, -speed);
        }
    return false;
}

public String RelativeScale = "";

public boolean alignOverride = false;

private static boolean aligned = false;

private static final double SCALE_TO_WALL_DISTANCE = 72;

// 35 is robot length with bumber
// 13 is cube length
private static final double ROBOT_LENGTH_WITH_CUBE = 48;

// 72- 48
// 24
private static final double ROBOT_TO_SCALE_DISTANCE = SCALE_TO_WALL_DISTANCE
        - ROBOT_LENGTH_WITH_CUBE;
}
