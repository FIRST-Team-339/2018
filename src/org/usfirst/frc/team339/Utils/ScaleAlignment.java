package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
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
 * TODO NOT TESTED; currently commented out in teleop
 * and this needs to be commented
 * 
 * Version of align to scale that should be used in teleop; calls the normal
 * alignToScale, but also properly sets the transmission and other variables to
 * properly use this function
 * 
 * @param button
 *            The button that will call alignToScale when pressed
 * 
 * @author C.R.
 */
public void alignToScaleByButtons (boolean button)
{

    if (button == true) // if the button is being pressed
        {
        // runs the first time after the button is pressed
        if (allowAlignment == false
                && alignButtonPressedLastTime == false)
            {
            allowAlignment = true;
            }

        if (allowAlignment == true)
            {
            // keep running align to scale until it returns true
            if (Hardware.scaleAlignment.alignToScale(.3, 3,
                    false) == true)
                {
                allowAlignment = false;
                }
            }
        }
    else
        {
        // set allowAlignmet to false so we stop locking out the driver
        // joysticks in teleop
        allowAlignment = false;
        // resets the transmission for teleop once, immediately after we stop
        // hold down the align button

        }
    // used like a momentary switch to prevent an infinite alignment loop
    alignButtonPressedLastTime = button;
}

// keeps track of whether or not the align by scale button was being pressed
// last time (for alignToScaleByButtons)
private boolean alignButtonPressedLastTime = false;

/**
 * TODO CHECK TO SEE IF THE OVERRIDE PARAMETER IS ACTUALLY NECESSARY
 * Align to proper distance with the scale
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
    // Cole's comment; the overrides are theoretically useless since wherever
    // alignToScale is called it should cancel when we let go of the button
    // that's calling it
    // if (override == true)
    // {
    // alignOverride = true;
    // }
    // if (alignOverride == true)
    // {
    // Hardware.transmission
    // .setForTeleop(Robot.KILROY_XIX_GEAR_2_SPEED);
    // aligned = false;
    // return false;
    // }
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

        // Hardware.transmission.drive(speed, speed);
        }
    // if to close to scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() > ROBOT_TO_SCALE_DISTANCE
                    - deadband
            && aligned == false && alignOverride == false)
        {
        SmartDashboard.putNumber("Speed", speed);

        RelativeScale = "Too close";

        // Hardware.transmission.drive(-speed, -speed);
        }
    return false;
}

public String RelativeScale = "";

// used to communicate with teleop whether or not to lock out the joysticks and
// let the autonomous code drive
public boolean allowAlignment = false;

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
