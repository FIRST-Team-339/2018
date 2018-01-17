/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Autonomous.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 13, 2015
// CREATED BY: Nathanial Lydick
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. Some of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// -----------------------------------------------------
// Periodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import org.usfirst.frc.team339.Hardware.Hardware;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 * An Autonomous class.
 * This class <b>beautifully</b> uses state machines in order to periodically
 * execute instructions during the Autonomous period.
 * 
 * This class contains all of the user code for the Autonomous part
 * of the
 * match, namely, the Init and Periodic code
 * 
 * 
 * @author Michael Andrzej Klaczynski
 * @written at the eleventh stroke of midnight, the 28th of January,
 *          Year of our LORD 2016. Rewritten ever thereafter.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Autonomous
{

/**
 * User Initialization code for autonomous mode should go here. Will run once
 * when the autonomous first starts, and will be followed immediately by
 * periodic().
 */
public static void init ()
{
    // Testing the game data the driver station provides us with
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();

    if (gameData.charAt(0) == 'L')
        {
        switchSide = Switch.LEFT;
        }
    else
        {
        switchSide = Switch.RIGHT;
        }

    System.out.println(switchSide);

} // end Init


// State of autonomous
public static enum State
    {
INIT, DELAY, CHOOSE_PATH, AUTOLINE, AUTOLINE_SCALE, AUTOLINE_EXCHANGE, CENTER_SWITCH, SWITCH_OR_SCALE_L, SWITCH_OR_SCALE_R, OFFSET_SWITCH, FINISH
    }

public static enum Switch
    {
LEFT, RIGHT, NULL
    }


public static Switch switchSide = Switch.NULL;

public static State autoState = State.INIT;

/**
 * User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void periodic ()
{
    if (Hardware.autoEnableSwitch.getPosition() == Value.kReverse)
        return;

    switch (autoState)
        {
        case INIT:
            // Reset and start the delay timer
            Hardware.autoTimer.reset();
            Hardware.autoTimer.start();
            break;

        case DELAY:
            // Delay using the potentiometer, from 0 to 5 seconds
            if (Hardware.autoTimer.get() >= Hardware.delayPot.get(0.0,
                    5.0))
                {
                autoState = State.CHOOSE_PATH;
                break;
                }

            break;
        case CHOOSE_PATH:
            // Middle position is for the auto line
            if (Hardware.autoEnableSwitch.getPosition() == Value.kOff)
                {
                // 1st position on 6pos is regular cross auto line
                // 2nd position is cross auto line and setup for scale (long
                // distance)
                // 3rd position is cross auto line and setup for exchange zone
                // in teleop
                // If anything else, the disable auto
                switch (Hardware.autoStateSwitch.getPosition())
                    {
                    case 0:
                        autoState = State.AUTOLINE;
                        break;
                    case 1:
                        autoState = State.AUTOLINE_SCALE;
                        break;
                    case 2:
                        autoState = State.AUTOLINE_EXCHANGE;
                        break;
                    default:
                        autoState = State.FINISH;
                    }
                }
            // Forward position is for the main autos
            else if (Hardware.autoEnableSwitch
                    .getPosition() == Value.kForward)
                {
                // 1st pos on 6pos switch is Center Switch with Vision
                // 2nd pos chooses switch or scale based on game data, on the
                // left side
                // 3rd pos chooses switch or scale based on game data, on the
                // right side
                // 4th pos delivers to switch from the offset-center position
                // If anything else, disable auto.
                switch (Hardware.autoStateSwitch.getPosition())
                    {
                    case 0:
                        autoState = State.CENTER_SWITCH;
                        break;
                    case 1:
                        autoState = State.SWITCH_OR_SCALE_L;
                        break;
                    case 2:
                        autoState = State.SWITCH_OR_SCALE_R;
                        break;
                    case 3:
                        autoState = State.OFFSET_SWITCH;
                        break;
                    default:
                        autoState = State.FINISH;
                    }
                }


            break;
        case AUTOLINE:
            Hardware.autoDrive.driveStraightInches(120, .5);
            autoState = State.FINISH;
            break;

        case AUTOLINE_SCALE:
            Hardware.autoDrive.driveStraightInches(207, .7);
            autoState = State.FINISH;
            break;

        case FINISH:

            break;

        default:
            break;
        }

}

/**
 * Delivers the cube to the switch based on vision processing.
 * 
 * @return
 *         Whether or not the robot has finished the action
 */
public static boolean centerSwitchPath ()
{
    switch (visionAuto)
        {
        case DRIVE_SIX_INCHES:
            if (Hardware.autoDrive.driveStraightInches(6, .5) == true)
                {

                }
            break;

        }
    return false;
}

/**
 * 
 */
public static centerState visionAuto = centerState.DRIVE_SIX_INCHES;

/**
 * Possible states for center vision autonomous
 * 
 * @author Becky Button
 *
 */
public static enum centerState
    {
DRIVE_SIX_INCHES, TURN_TOWARDS_LEFT_SIDE, TURN_TOWARDS_RIGHT_SIDE, STOP_BEFORE_PICTURE, DRIVE_BY_CAMERA, DRIVE_BY_US, DEPLOY_INTAKE, SPIT_OUT_CUBE, BRAKE, DONE
    }


/*
 * ================================
 * Constants
 * ================================
 */

// INIT


// DELAY


// CHOOSE_PATH


// AUTOLINE


// AUTOLINE_SCALE


// AUTOLINE_EXCHANGE


// CENTER_SWITCH

private final double ANGLE_TOWARDS_LEFT = 35;

private final double ANGLE_TOWARDS_RIGHT = 24.5;


// SWITCH_OR_SCALE_L


// SWITCH_OR_SCALE_R


// OFFSET_SWITCH


// FINISH



} // end class

