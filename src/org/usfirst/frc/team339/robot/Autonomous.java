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
    if (Hardware.leftRightDisableAutoSwitch.getPosition() == Value.kOff)
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
            autoState = State.DELAY;
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
            /*
             * States:
             * 0 = AUTOLINE
             * 1 = AUTOLINE THEN SCALE
             * 2 = AUTOLINE THEN EXCHANGE
             * 3 = CENTER SWITCH W/ VISION
             * 4 = SWITCH OR SCALE W/ GAME DATA
             * 5 = OFFSET SWITCH DROP OFF
             */

            switch (Hardware.autoSixPosSwitch.getPosition())
                {
                case 0:
                    autoState = State.AUTOLINE;
                    break;
                case 1:
                    autoState = State.AUTOLINE_SCALE;
                    break;
                case 2:
                    // Depends on whether left or right is selected
                    if (Hardware.leftRightDisableAutoSwitch
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
                    if (Hardware.leftRightDisableAutoSwitch
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
            if (autolinePath() == true)
                {
                autoState = State.FINISH;
                }
            break;
        case AUTOLINE_SCALE:
            if (Hardware.autoDrive.driveStraightInches(207,
                    .7) == false)
                {
                autoState = State.FINISH;
                }
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
 * Receives the game data involving the switch sides and scale side
 * 
 * @return
 *         Switch side either RIGHT or LEFT
 */
public static GameData grabData (GameDataType dataType)
{
    // Testing the game data the driver station provides us with
    String gameData = DriverStation.getInstance()
            .getGameSpecificMessage();

    // Stay in this state if there are not 3 letters in the game data
    // provided
    if (gameData.length() < 3)
        // return

        if (dataType == GameDataType.SWITCH
                && gameData.charAt(0) == 'L')
            {
            return GameData.LEFT;
            }
        else if (dataType == GameDataType.SWITCH
                && gameData.charAt(0) == 'R')
            {
            return GameData.RIGHT;
            }

    if (dataType == GameDataType.SCALE && gameData.charAt(1) == 'L')
        {
        return GameData.LEFT;
        }
    else if (dataType == GameDataType.SCALE
            && gameData.charAt(1) == 'R')
        {
        return GameData.RIGHT;
        }

    return GameData.NULL;
}

private enum GameDataType
    {
SWITCH, SCALE
    }

/**
 * crosses the autoline and stops
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean autolinePath ()
{
    if (Hardware.autoDrive.driveStraightInches(
            DISTANCE_TO_CROSS_AUTOLINE,
            DRIVE_SPEED) == true)
        return true;
    return false;
}

/**
 * crosses the autoline and drives near to scale and stops
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean autolinePathToScale ()
{
    if (Hardware.autoDrive.driveStraightInches(
            DISTANCE_TO_CROSS_AUTOLINE_AND_GO_TO_SCALE,
            DRIVE_SPEED) == true)
        {
        return true;
        }
    return false;
}

/**
 * crosses the autoline and stops
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean driveBackAcrossAutoline ()
{
    if (Hardware.autoDrive.driveStraightInches(
            DISTANCE_BACK_ACROSS_AUTOLINE,
            -DRIVE_SPEED) == true)
        return true;
    return false;
}

/**
 * drives to the exchange zone
 * 
 * @return
 *         boolean Whether or not the robot has finished the path
 */
public static boolean leftDriveToExchange ()
{

    if (Hardware.autoDrive.driveStraightInches(
            LEFT_DISTANCE_TO_EXCHANGE,
            DRIVE_SPEED) == true)
        return true;
    return false;
}

/**
 * drives to the exchange zone
 * 
 * @return
 *         boolean Whether or not the robot has finished the path
 */
public static boolean rightDriveToExchange ()
{

    if (Hardware.autoDrive.driveStraightInches(
            RIGHT_DISTANCE_TO_EXCHANGE,
            DRIVE_SPEED) == true)
        return true;
    return false;
}

/**
 * turns 90 degrees to the right
 * 
 * @return
 *         boolean Whether or not the robot has finished the path
 */
public static boolean turn90DegreesRight ()
{

    if (Hardware.autoDrive.turnDegrees(LEFT_SIDE_TURN_TOWARDS_EXCHANGE,
            TURN_SPEED) == true)
        return true;
    return false;
}

/**
 * turns 90 degrees to the left
 * 
 * @return
 *         boolean Whether or not the robot has finished the path
 */
public static boolean turn90DegreesLeft ()
{

    if (Hardware.autoDrive.turnDegrees(RIGHT_SIDE_TURN_TOWARDS_EXCHANGE,
            TURN_SPEED) == true)
        return true;
    return false;
}


public static leftExchangeState leftExchangeAuto = leftExchangeState.DONE;

/**
 * Possible states for left exchange autonomous
 * 
 * @author Ashley Espeland
 *
 */
public static enum leftExchangeState
    {
DRIVE_ACROSS_AUTOLINE, DRIVE_BACK_ACROSS_AUTOLINE, TURN_90_DEGREES_RIGHT, DRIVE_TO_EXCHANGE, DONE
    }



/**
 * Crosses the auto line and returns to the exchange zone to setup for teleop.
 * Starts in the left corner.
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean leftAutoLineExchangePath ()
{
    switch (leftExchangeAuto)
        {
        case DRIVE_ACROSS_AUTOLINE:
            if (autolinePath() == true)
                {
                leftExchangeAuto = leftExchangeState.DRIVE_BACK_ACROSS_AUTOLINE;
                }
            break;

        case DRIVE_BACK_ACROSS_AUTOLINE:
            if (driveBackAcrossAutoline() == true)
                {
                leftExchangeAuto = leftExchangeState.TURN_90_DEGREES_RIGHT;
                }
            break;

        case TURN_90_DEGREES_RIGHT:
            if (turn90DegreesRight() == true)
                {
                leftExchangeAuto = leftExchangeState.DRIVE_TO_EXCHANGE;
                }
            break;

        case DRIVE_TO_EXCHANGE:
            if (leftDriveToExchange() == true)
                {
                leftExchangeAuto = leftExchangeState.DONE;
                }
            break;

        case DONE:

            break;

        }


    return false;
}

public static rightExchangeState rightExchangeAuto = rightExchangeState.DONE;

/**
 * Possible states for right exchange autonomous
 * 
 * @author Ashley Espeland
 *
 */
public static enum rightExchangeState
    {
DRIVE_ACROSS_AUTOLINE, DRIVE_BACK_ACROSS_AUTOLINE, TURN_90_DEGREES_LEFT, DRIVE_TO_EXCHANGE, DONE
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
switch (rightExchangeAuto)
{
case DRIVE_ACROSS_AUTOLINE:
    if (autolinePath() == true)
        {
        rightExchangeAuto = rightExchangeState.DRIVE_BACK_ACROSS_AUTOLINE;
        }
    break;

case DRIVE_BACK_ACROSS_AUTOLINE:
    if (driveBackAcrossAutoline() == true)
        {
        rightExchangeAuto = rightExchangeState.TURN_90_DEGREES_LEFT;
        }
    break;

case TURN_90_DEGREES_LEFT:
    if (turn90DegreesLeft() == true)
        {
        rightExchangeAuto = rightExchangeState.DRIVE_TO_EXCHANGE;
        }
    break;

case DRIVE_TO_EXCHANGE:
    if (rightDriveToExchange() == true)
        {
        rightExchangeAuto = rightExchangeState.DONE;
        }
    break;

case DONE:

    break;

}

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
                visionAuto = centerState.BRAKE_1;
                }
            break;
        case BRAKE_1:
            if (Hardware.autoDrive.brake() == true)
                {
                visionAuto = centerState.GRAB_DATA;
                }
            break;
        case GRAB_DATA:
            if (grabData(GameDataType.SWITCH) == GameData.LEFT)
                {
                visionAuto = centerState.TURN_TOWARDS_LEFT_SIDE;
                }
            else if (grabData(
                    GameDataType.SWITCH) == GameData.RIGHT)
                {
                visionAuto = centerState.TURN_TOWARDS_RIGHT_SIDE;
                }
            else
                {
                visionAuto = centerState.DONE;
                }
            break;
        case TURN_TOWARDS_LEFT_SIDE:
            if (Hardware.autoDrive.turnDegrees(ANGLE_TOWARDS_LEFT,
                    AUTO_SPEED_VISION))
                {
                visionAuto = centerState.BRAKE_2_L;
                }
            break;
        case TURN_TOWARDS_RIGHT_SIDE:
            if (Hardware.autoDrive.turnDegrees(ANGLE_TOWARDS_RIGHT,
                    AUTO_SPEED_VISION))
                {
                visionAuto = centerState.BRAKE_2_R;
                }
        case BRAKE_2_L:
            if (Hardware.autoDrive.brake())
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_LEFT;
                }
            break;
        case BRAKE_2_R:
            if (Hardware.autoDrive.brake())
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_RIGHT;
                }
            break;
        case DRIVE_STRAIGHT_TO_SWITCH_LEFT:
            if (Hardware.autoDrive.driveStraightInches(
                    DRIVE_NO_CAMERA_LEFT, AUTO_SPEED_VISION))
                {

                }
            break;
        case DRIVE_STRAIGHT_TO_SWITCH_RIGHT:
            if (Hardware.autoDrive.driveStraightInches(
                    DRIVE_NO_CAMERA_RIGHT, AUTO_SPEED_VISION))
                {
                }
            break;


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
DRIVE_SIX_INCHES, BRAKE_1, GRAB_DATA, TURN_TOWARDS_LEFT_SIDE, TURN_TOWARDS_RIGHT_SIDE, BRAKE_2_L, BRAKE_2_R, DRIVE_STRAIGHT_TO_SWITCH_LEFT, DRIVE_STRAIGHT_TO_SWITCH_RIGHT, DONE
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
    System.out.println("Current State: " + currentSwitchOrScaleState);
    switch (currentSwitchOrScaleState)
        {
        case INIT:

            break;
        case DRIVE1:
            // FIRST driveInches: drive forward to switch
            if (Hardware.autoDrive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[0], DRIVE_SPEED))
                {
                // If the switch IS on the right side, brake before turning
                // towards it
                if (grabData(GameDataType.SWITCH) == GameData.RIGHT)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE1;
                else
                    // If not, then keep driving forwards.
                    currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE2;

                }
            break;
        case BRAKE_DRIVE1:
            // Brake before turning towards the switch
            if (Hardware.autoDrive.brake())
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN1;
            break;
        case TURN1:
            // Turn towards the switch
            if (Hardware.autoDrive.turnDegrees(-90, TURN_SPEED))
                currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_TURN1;

            break;
        case BRAKE_TURN1:
            // Brake after turning, and before going to the switch with
            // ultrasonic
            if (Hardware.autoDrive.brake())
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_WITH_ULTRSNC;
            break;
        case RAISE_ARM1:
            // Raise the arm for the switch after turning towards it, but before
            // driving to the wall.
            if (Hardware.cubeManipulator.moveLiftDistance(
                    SWITCH_LIFT_HEIGHT, FORKLIFT_SPEED))
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_WITH_ULTRSNC;
            break;
        case DRIVE_WITH_ULTRSNC:
            // Drive to the switch until the ultrasonic tells us to stop
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() < SWITCH_OR_SCALE_ULTRSNC_DISTANCE)
                currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_ULTRSNC;

            break;
        case BRAKE_ULTRSNC:
            // Brake after driving using the ultrasonic
            if (Hardware.autoDrive.brake())
                {
                Hardware.autoTimer.stop();
                Hardware.autoTimer.reset();
                Hardware.autoTimer.start();
                currentSwitchOrScaleState = SwitchOrScaleStates.EJECT_CUBE;
                }
            break;
        case EJECT_CUBE:
            // Eject the cube onto the switch platform.
            Hardware.cubeIntakeMotor.set(-INTAKE_SPEED);
            if (Hardware.autoTimer.get() > INTAKE_EJECT_TIME)
                {
                Hardware.cubeManipulator.stopIntake();
                Hardware.autoTimer.stop();
                currentSwitchOrScaleState = SwitchOrScaleStates.FINISH;
                }

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
INIT, DRIVE1, BRAKE_DRIVE1, TURN1, BRAKE_TURN1, RAISE_ARM1, DRIVE_WITH_ULTRSNC, BRAKE_ULTRSNC, EJECT_CUBE, DRIVE2, BRAKE_DRIVE2, TURN2, BRAKE_TURN2, DRIVE3, BRAKE_DRIVE3, TURN3, DRIVE4, BRAKE_DRIVE4, TURN4, BRAKE_B4_FINISH, FINISH
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

private static final double INTAKE_EJECT_TIME = 1;// Seconds

private static final int SWITCH_LIFT_HEIGHT = 24;// Inches

private static final double FORKLIFT_SPEED = .5;

private static final double INTAKE_SPEED = 1;

// INIT


// DELAY


// CHOOSE_PATH


// AUTOLINE
private final static int DISTANCE_TO_CROSS_AUTOLINE = 120;

// AUTOLINE_SCALE
private final static int DISTANCE_TO_CROSS_AUTOLINE_AND_GO_TO_SCALE = 207;

// AUTOLINE_EXCHANGE
private final static int DISTANCE_BACK_ACROSS_AUTOLINE = 100;

private final static int LEFT_DISTANCE_TO_EXCHANGE = 58;

private final static int LEFT_SIDE_TURN_TOWARDS_EXCHANGE = 90;

private final static int RIGHT_SIDE_TURN_TOWARDS_EXCHANGE = -90;

private final static int RIGHT_DISTANCE_TO_EXCHANGE = 130;

// CENTER_SWITCH
private final static int DRIVE_NO_CAMERA_LEFT = 118;

private final static int DRIVE_NO_CAMERA_RIGHT = 116;

// TODO change all of these to be real numbers -- just placeholders for right
// now
private final static int ANGLE_TOWARDS_LEFT = 35;

private final static int ANGLE_TOWARDS_RIGHT = 24;

// TODO change for actual auto speed
private final static double AUTO_SPEED_VISION = .5;


// SWITCH_OR_SCALE_L
private static final int[] SWITCH_OR_SCALE_DRIVE_DISTANCE = new int[]
    {133, 67, 31, 169};

private static final int SWITCH_OR_SCALE_ULTRSNC_DISTANCE = 4;// Inches

// SWITCH_OR_SCALE_R


// OFFSET_SWITCH


// FINISH



} // end class

