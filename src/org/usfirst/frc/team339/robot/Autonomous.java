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
import edu.wpi.first.wpilibj.Relay;
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
INIT, DELAY, CHOOSE_PATH, AUTOLINE, AUTOLINE_SCALE, AUTOLINE_EXCHANGE_L, AUTOLINE_EXCHANGE_R, CENTER_SWITCH, SWITCH_OR_SCALE_L, SWITCH_OR_SCALE_R, OFFSET_SWITCH, FINISH
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
    // Disable autonomous
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
                // in teleop from the left side
                // 4th position is cross auto line and setup for exchange zone
                // in teleop from the right side
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
                        autoState = State.AUTOLINE_EXCHANGE_L;
                        break;
                    case 3:
                        autoState = State.AUTOLINE_EXCHANGE_R;
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
            // TODO THIS IS WRONG! fix this ashley. pls
            Hardware.autoDrive.driveStraightInches(120, .5);
            autoState = State.FINISH;
            break;
        case AUTOLINE_SCALE:
            // TODO THIS IS WRONG TOO! fix this ashley. pls
            Hardware.autoDrive.driveStraightInches(207, .7);
            autoState = State.FINISH;
            break;
        case AUTOLINE_EXCHANGE_L:
            if (leftAutoLineExchangePath() == true)
                autoState = State.FINISH;
            break;
        case AUTOLINE_EXCHANGE_R:
            if (rightAutoLineExchangePath() == true)
                autoState = State.FINISH;
            break;
        case CENTER_SWITCH:
            if (centerSwitchPath() == true)
                autoState = State.FINISH;
            break;
        case SWITCH_OR_SCALE_L:
            if (leftSwitchOrScalePath() == true)
                autoState = State.FINISH;
            break;
        case SWITCH_OR_SCALE_R:
            if (rightSwitchOrScalePath() == true)
                autoState = State.FINISH;
            break;
        case OFFSET_SWITCH:
            if (offsetSwitchPath() == true)
                autoState = State.FINISH;
            break;
        case FINISH:
            Hardware.tractionDrive.stop();
            Hardware.autoTimer.stop();
            Hardware.autoTimer.reset();
            break;

        default:
            break;
        }

}

/**
 * Crosses the auto line and returns to the exchange zone to setup for teleop.
 * Starts in the left corner.
 * 
 * @return
 */
public static boolean leftAutoLineExchangePath ()
{

    return false;
}

/**
 * Crosses the auto line and returns to the exchange zone to setup for teleop.
 * Starts in the right corner.
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean rightAutoLineExchangePath ()
{

    return false;
}

/**
 * Delivers the cube to the switch based on vision processing.
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean centerSwitchPath ()
{

    return false;
}

/**
 * Delivers the cube to the switch if its on the left side.. If not, then setup
 * to either the left or right scale, all based on game data from the driver
 * station
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean leftSwitchOrScalePath ()
{


    return false;
}

/**
 * Delivers the cube to the switch if it's on the right side... If not, then
 * setup to either the left or right scale, all based on game data from the
 * driver station.
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean rightSwitchOrScalePath ()
{

    return false;
}

/**
 * Delivers the cube if another robot chooses to deliver in the center. Goes
 * around the switch to deliver on the end.
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean offsetSwitchPath ()
{

    return false;
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


// SWITCH_OR_SCALE_L


// SWITCH_OR_SCALE_R


// OFFSET_SWITCH


// FINISH



} // end class

