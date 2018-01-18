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
    // Disable auto
    if (Hardware.autoSelectorSwitch.getPosition() == Value.kOff)
        autoState = State.FINISH;
} // end Init


// State of autonomous
public static enum State
    {
INIT, DELAY, GRAB_DATA, CHOOSE_PATH, AUTOLINE, AUTOLINE_SCALE, AUTOLINE_EXCHANGE_L, AUTOLINE_EXCHANGE_R, CENTER_SWITCH, SWITCH_OR_SCALE_L, SWITCH_OR_SCALE_R, OFFSET_SWITCH, FINISH
    }

public static enum GameData
    {
LEFT, RIGHT, NULL
    }


public static GameData switchSide = GameData.NULL;

public static GameData scaleSide = GameData.NULL;

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
                autoState = State.GRAB_DATA;
                break;
                }

            break;

        case GRAB_DATA:

            // Testing the game data the driver station provides us with
            String gameData = DriverStation.getInstance()
                    .getGameSpecificMessage();

            // Stay in this state if there are not 3 letters in the game data
            // provided
            if (gameData.length() < 3)
                break;

            if (gameData.charAt(0) == 'L')
                {
                switchSide = GameData.LEFT;
                }
            else
                {
                switchSide = GameData.RIGHT;
                }

            if (gameData.charAt(1) == 'L')
                {
                scaleSide = GameData.LEFT;
                }
            else
                {
                scaleSide = GameData.RIGHT;
                }

            System.out.println(switchSide);

            autoState = State.CHOOSE_PATH;
            break;
        case CHOOSE_PATH:
            /*
             * States:
             * 0 = AUTOLINE
             * 1 = AUTOLINE THEN SCALE
             * 2 = AUTOLINE THEN EXCHANGE
             * 3 = CENTER SWITCH W/ VISION
             * 4 = SWITCH OR SCALE W/ GAME DATA
             * 5 = OFFSET SWITCH DROP OFF
             */

            switch (Hardware.autoStateSwitch.getPosition())
                {
                case 0:
                    autoState = State.AUTOLINE;
                    break;
                case 1:
                    autoState = State.AUTOLINE_SCALE;
                    break;
                case 2:
                    // Depends on whether left or right is selected
                    if (Hardware.autoSelectorSwitch
                            .getPosition() == Value.kForward)
                        autoState = State.AUTOLINE_EXCHANGE_L;
                    else
                        autoState = State.AUTOLINE_EXCHANGE_R;
                    break;
                case 3:
                    autoState = State.CENTER_SWITCH;
                    break;
                case 4:
                    // Depends on whether left or right is selected
                    if (Hardware.autoSelectorSwitch
                            .getPosition() == Value.kForward)
                        autoState = State.SWITCH_OR_SCALE_L;
                    else
                        autoState = State.SWITCH_OR_SCALE_R;
                    break;
                case 5:
                    autoState = State.OFFSET_SWITCH;
                    break;
                default:
                    // If for some reason we failed, then disable.
                    autoState = State.FINISH;
                    break;
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
 * Delivers the cube to the switch based on vision processing.
 * 
 * @return
 *         Whether or not the robot has finished the path
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
 * Crosses the auto line and returns to the exchange zone to setup for teleop.
 * Starts in the left corner.
 * 
 * @return
 *         Whether or not the robot has finished the action
 */
public static boolean centerSwitchPath ()
{
    switch (visionAuto)
        {
        case DRIVE_SIX_INCHES:
            if (Hardware.autoDrive.driveStraightInches(6,
                    AUTO_SPEED_VISION) == true)
                {
                if (Hardware.autoDrive.brake() == true)
                    {
                    if (switchSide == GameData.LEFT)
                        {
                        visionAuto = centerState.TURN_TOWARDS_LEFT_SIDE;
                        }
                    else if (switchSide == GameData.RIGHT)
                        {
                        visionAuto = centerState.TURN_TOWARDS_RIGHT_SIDE;
                        }
                    else
                        {
                        visionAuto = centerState.DONE;
                        }
                    }
                }
            break;
        case TURN_TOWARDS_LEFT_SIDE:
            if (Hardware.autoDrive.turnDegrees(ANGLE_TOWARDS_LEFT,
                    AUTO_SPEED_VISION))
                {
                if (Hardware.autoDrive.brake() == true)
                    {
                    }
                }
            break;
        case TURN_TOWARDS_RIGHT_SIDE:
            if (Hardware.autoDrive.turnDegrees(ANGLE_TOWARDS_RIGHT,
                    AUTO_SPEED_VISION))
                {
                if (Hardware.autoDrive.brake() == true)
                    {

                    }
                }

        }
    return false;
}

public static centerState visionAuto = centerState.DRIVE_SIX_INCHES;

/**
 * Possible states for center vision autonomous
 * 
 * @author Becky Button
 *
 */
public static enum centerState
    {
DRIVE_SIX_INCHES, TURN_TOWARDS_LEFT_SIDE, TURN_TOWARDS_RIGHT_SIDE, DONE
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
    switch (currentSwitchOrScaleState)
        {
        case INIT:

            break;
        case DRIVE1:
            if (Hardware.autoDrive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[0], DRIVE_SPEED))
                {
                if (switchSide == GameData.RIGHT)

                    // TODO Pick up where I left off
                    currentSwitchOrScaleState = SwitchOrScaleStates.TURN1;
                }
            break;
        case TURN1:
            break;
        case DRIVE_WITH_ULTRSNC:

            break;
        case DRIVE2:

            break;
        case TURN2:

            break;
        case DRIVE3:

            break;
        case TURN3:

            break;
        case DRIVE4:

            break;
        case TURN4:

            break;

        default:
        case FINISH:

            break;
        }
    return false;
}


private static SwitchOrScaleStates currentSwitchOrScaleState = SwitchOrScaleStates.INIT;

private static enum SwitchOrScaleStates
    {
INIT, DRIVE1, TURN1, DRIVE_WITH_ULTRSNC, DRIVE2, TURN2, DRIVE3, TURN3, DRIVE4, TURN4, FINISH
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

private static final double DRIVE_SPEED = .6;

private static final double TURN_SPEED = .5;

// INIT


// DELAY


// CHOOSE_PATH


// AUTOLINE


// AUTOLINE_SCALE


// AUTOLINE_EXCHANGE


// CENTER_SWITCH
private final static double DISTANCE_TO_LEFT_TARGET = 118;

private final static double DISTANCE_TO_RIGHT_TARGET = 116;

// TODO change all of these to be real numbers -- just placeholders for right
// now
private final static int ANGLE_TOWARDS_LEFT = 35;

private final static int ANGLE_TOWARDS_RIGHT = 24;

// TODO change for actual auto speed
private final static double AUTO_SPEED_VISION = .5;


// SWITCH_OR_SCALE_L
private static final int[] SWITCH_OR_SCALE_DRIVE_DISTANCE = new int[]
    {133, 67, 31, 169};

// SWITCH_OR_SCALE_R


// OFFSET_SWITCH


// FINISH



} // end class

