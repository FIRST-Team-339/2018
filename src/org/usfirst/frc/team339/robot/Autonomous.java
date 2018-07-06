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
import org.usfirst.frc.team339.Utils.CubeManipulator;
import org.usfirst.frc.team339.Utils.drive.Drive.BrakeType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------
    Hardware.leftDriveMotor.setSafetyEnabled(false);
    Hardware.rightDriveMotor.setSafetyEnabled(false);
    Hardware.liftingMotor.setSafetyEnabled(false);
    Hardware.cubeIntakeMotor.setSafetyEnabled(false);
    Hardware.intakeDeployArm.setSafetyEnabled(false);

    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.liftingEncoder.reset();
    Hardware.intakeDeployEncoder.reset();
    // Disable auto

    if (Hardware.disableAutonomousSwitch.isOn() == true)
        autoState = State.FINISH;

    Hardware.drive
            .setDefaultAcceleration(DRIVE_STRAIGHT_ACCELERATION_TIME);

    System.out.println("Game Data: "
            + Hardware.driverStation.getGameSpecificMessage());
} // end Init

/**
 * State of autonomous as a whole; mainly for init, delay, finish, and choosing
 * which autonomous path is being used
 */
public static enum State
    {
INIT, DELAY, CHOOSE_PATH, AUTOLINE, AUTOLINE_SCALE, AUTOLINE_EXCHANGE_L, AUTOLINE_EXCHANGE_R, CENTER_SWITCH, SWITCH_OR_SCALE_L, SWITCH_OR_SCALE_R, OFFSET_SWITCH, FINISH
    }

public static enum Position
    {
LEFT, RIGHT, NULL
    }

// variable that controls the state of autonomous as a whole (init, delay
// which path is being used, etc.)
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
    SmartDashboard.putString("Overall Auto state",
            autoState.toString());
    SmartDashboard.putString("Vision auto state",
            visionAuto.toString());
    // calls the forklift to update itself, allowing us to use the
    // forklift state machine; necessary for the forklift to work properly
    Hardware.cubeManipulator.masterUpdate();
    // prints the main state of autonomous (as a whole) we're in
    // System.out.println("Main State: " + autoState);
    // calls the print statements from Teleop
    Teleop.printStatements();
    // Main switch statement of auto
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
            // once finished, stop the timer and go to the next state

            if (Hardware.autoTimer.get() >= Hardware.delayPot.get(0.0,
                    5.0))
                {
                autoState = State.CHOOSE_PATH;
                Hardware.autoTimer.stop();
                break;
                }
            break;

        case CHOOSE_PATH:
            /*
             * States: 0 = AUTOLINE 1 = AUTOLINE THEN SCALE 2 = AUTOLINE THEN
             * EXCHANGE 3 = CENTER SWITCH W/ VISION 4 = SWITCH OR SCALE W/ GAME
             * DATA 5 = OFFSET SWITCH DROP OFF
             */
            // decide which auto path we're using based off of the two auto
            // switches

            switch (Hardware.autoSixPosSwitch.getPosition())
                {
                case 0:
                    // drive across autoline
                    autoState = State.AUTOLINE;
                    break;
                case 1:
                    // drives and sets up at scale
                    autoState = State.AUTOLINE_SCALE;
                    break;
                case 2:
                    // cross autoline, then go to the exchange
                    // two versions depending on whether left or right is
                    // selected
                    if (Hardware.leftAutoSwitch.isOn() == true)
                        autoState = State.AUTOLINE_EXCHANGE_L;
                    else
                        autoState = State.AUTOLINE_EXCHANGE_R;
                    break;
                case 3:
                    // start in the middle between the two switch sides;
                    // use vision and game data to drive to the correct switch
                    // side and drop off the cube
                    autoState = State.CENTER_SWITCH;
                    break;
                case 4:
                    // if the robot starts on the right side for the switch,
                    // drop of the cube
                    if (Hardware.leftAutoSwitch.isOn() == true)
                        {
                        // if either the switch or scale is on our side, then
                        // run the switchOrScalePath
                        if (grabData(
                                GameDataType.SWITCH) == Position.LEFT
                                || grabData(
                                        GameDataType.SCALE) == Position.LEFT)
                            autoState = State.SWITCH_OR_SCALE_L;
                        // else Switch or scale is NOT on our side? Setup for
                        // exchange.
                        else
                            autoState = State.AUTOLINE_EXCHANGE_L;
                        }
                    else
                        {
                        // if either the switch or scale is on our side, then
                        // run the switchOrScalePath
                        if (grabData(
                                GameDataType.SWITCH) == Position.RIGHT
                                || grabData(
                                        GameDataType.SCALE) == Position.RIGHT)
                            autoState = State.SWITCH_OR_SCALE_R;
                        else
                            // Switch or scale is NOT on our side? Setup for
                            // exchange.
                            autoState = State.AUTOLINE_EXCHANGE_R;
                        }
                    break;
                case 5:
                    // Delivers the cube if another robot chooses to deliver in
                    // the center. Goes
                    // around the switch to deliver on the end.
                    autoState = State.OFFSET_SWITCH;
                    break;
                default:

                    // If for some reason we failed to properly set the auto
                    // State, then print we reached the default case and
                    // disable.
                    System.out.println(
                            "REACHED THE DEFAULT CASE FOR the six position switch)");
                    autoState = State.FINISH;
                    break;
                }
            break;
        case AUTOLINE:
            // calls method that runs that path
            if (autolinePath() == true)
                {
                autoState = State.FINISH;
                }
            break;
        case AUTOLINE_SCALE:
            // calls method that runs that path
            if (autoLineScalePath() == true)
                {
                autoState = State.FINISH;
                }
            break;
        case AUTOLINE_EXCHANGE_L:
            // calls method that runs that path
            if (leftAutoLineExchangePath() == true)
                autoState = State.FINISH;
            break;
        case AUTOLINE_EXCHANGE_R:
            // calls method that runs that path
            if (rightAutoLineExchangePath() == true)
                autoState = State.FINISH;
            break;
        case CENTER_SWITCH:
            // calls method that runs vision path
            if (centerSwitchPath() == true)
                autoState = State.FINISH;
            break;
        case SWITCH_OR_SCALE_L:
            // calls method that runs that path
            if (switchOrScalePath(Position.LEFT) == true)
                autoState = State.FINISH;
            break;
        case SWITCH_OR_SCALE_R:
            // calls method that runs that path
            if (switchOrScalePath(Position.RIGHT) == true)
                autoState = State.FINISH;
            break;
        case OFFSET_SWITCH:
            // calls method that runs that path
            if (offsetSwitchPath() == true)
                autoState = State.FINISH;
            break;
        case FINISH:

            // end of autonomous; stops and resets the autotimer
            Hardware.transmission.stop();
            // Hardware.autoTimer.stop();
            // Hardware.autoTimer.reset();
            break;

        default:

            // if something goes wrong, stop autonomous and go straight
            // to FINISH
            Hardware.transmission.stop();
            // Hardware.autoTimer.stop();
            // Hardware.autoTimer.reset();
            autoState = State.FINISH;
            break;
        }

}

/**
 * Receives the game data involving the switch sides and scale side
 * 
 * @param dataType
 *            - denotes either SWITCH or SCALE
 * @return
 *         Switch side either RIGHT or LEFT
 */
public static Position grabData (GameDataType dataType)
{
    Position position = Position.NULL;

    // gets the game data the driver station provides us with
    String gameData = DriverStation.getInstance()
            .getGameSpecificMessage();

    // Stay in this state if there are not 3 letters in the game data
    // provided
    if (gameData.length() < 3)
        position = Position.NULL;
    // return null

    // sets the switch position based on the game data
    if (dataType == GameDataType.SWITCH && gameData.charAt(0) == 'L')
        {
        position = Position.LEFT;
        }
    else if (dataType == GameDataType.SWITCH
            && gameData.charAt(0) == 'R')
        {
        position = Position.RIGHT;
        }

    // sets the scale position based on the game data
    if (dataType == GameDataType.SCALE && gameData.charAt(1) == 'L')
        {
        position = Position.LEFT;
        }
    else if (dataType == GameDataType.SCALE
            && gameData.charAt(1) == 'R')
        {
        position = Position.RIGHT;
        }
    // FORCING for testing
    // position = Position.LEFT;
    // position = Position.RIGHT;
    return position;
}

/**
 * Enum used by the grabData function for specifying if we want to know
 * the game data for the (our) switch, or the scale;
 * does not currently have something for the opponent's switch
 */
public enum GameDataType
    {
SWITCH, SCALE
    }

/**
 * Autonomous path for just driving across the auto line
 * 
 * 
 * @return
 *         Whether or not the path has finished.
 */

public static boolean autolinePath ()
{
    // System.out.println("autoline path state : " + currentAutolineState);
    SmartDashboard.putNumber("Left Encoder",
            Hardware.leftFrontDriveEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder",
            Hardware.rightFrontDriveEncoder.getDistance());

    SmartDashboard.putNumber("Left Motor",
            Hardware.leftDriveMotor.get());
    SmartDashboard.putNumber("Right Motor",
            Hardware.rightDriveMotor.get());
    // autoline switch statement
    switch (currentAutolineState)
        {
        // Initialize anything necessary
        case PATH_INIT:
            Hardware.drive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            currentAutolineState = AutolinePathStates.DRIVE1;
            break;
        case DRIVE1:
            // Drive across the line
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE - Hardware.drive
                            .getBrakeStoppingDistance(),
                    DRIVE_SPEED, DRIVE_STRAIGHT_ACCELERATION_TIME,
                    true) == true)
                currentAutolineState = AutolinePathStates.BRAKE1;
            break;
        case BRAKE1:
            // Brake after driving across the line

            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                currentAutolineState = AutolinePathStates.DEPLOY;
            break;
        case DEPLOY:
            // if (Hardware.cubeManipulator.deployCubeIntake(false))
            // {
            currentAutolineState = AutolinePathStates.FINISH;
            // }
            break;
        default:
            // if something goes wrong, print we reached default and fall
            // through to FINISH's case
            System.out.println(
                    "REACHED THE DEFAULT CASE FOR autolinePath()");
        case FINISH:
            // Stop drive motors and end auto
            Hardware.transmission.stop();
            return true;
        }

    return false;
}

// enum for the states in the autolinePath and autolineScalePath autonomi
private static enum AutolinePathStates
    {
PATH_INIT, DRIVE1, BRAKE1, FINISH, DEPLOY
    }

/**
 * Drive across the auto line, and farther for the scale in teleop.
 * 
 * @return
 *         whether or not the path has finished
 */
public static boolean autoLineScalePath ()
{
    // autoline switch statement
    switch (currentAutolineState)
        {
        // Initialize anything necessary
        case PATH_INIT:
            Hardware.drive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            currentAutolineState = AutolinePathStates.DRIVE1;
            break;
        case DRIVE1:
            // Drive across the auto line, and a little farther to be closer to
            // the scale
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE_AND_GO_TO_SCALE
                            - Hardware.drive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED, DRIVE_STRAIGHT_ACCELERATION_TIME,
                    true) == true)
                currentAutolineState = AutolinePathStates.BRAKE1;
            break;
        case BRAKE1:
            // Brake after driving across the line
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                currentAutolineState = AutolinePathStates.DEPLOY;
            break;

        case DEPLOY:
            Hardware.cubeManipulator.deployCubeIntake(false);
            currentAutolineState = AutolinePathStates.FINISH;
            break;

        default: // prints we reached the default case, then fall through to
                 // FINISH
            System.out.println(
                    "REACHED THE DEFAULT CASE FOR autoLineScalePath()");
        case FINISH:
            // Stop drive motors and end auto
            Hardware.transmission.stop();
            return true;
        }
    return false;
}

private static AutolinePathStates currentAutolineState = AutolinePathStates.PATH_INIT;

public static leftExchangeState leftExchangeAuto = leftExchangeState.PATH_INIT;

/**
 * Possible states for left exchange autonomous
 * 
 * @author Ashley Espeland
 *
 */
public static enum leftExchangeState
    {
PATH_INIT, DRIVE_ACROSS_AUTOLINE, BRAKE_B4_DRIVE_BACK_ACROSS_AUTOLINE, TIMER_DELAY_ONE, DRIVE_BACK_ACROSS_AUTOLINE, BRAKE_B4_TURN, TIMER_DELAY_TWO, TURN_90_DEGREES_RIGHT, BRAKE_AFTER_TURN, TIMER_DELAY_THREE, DRIVE_TO_EXCHANGE, BRAKE_B4_DEPLOY, TIMER_DELAY_FOUR, DEPLOY, DONE
    }

/**
 * Crosses the auto line and returns to the exchange zone to setup for teleop.
 * Starts in the left corner.
 * 
 * @return
 *         Whether or not the robot has finished the path
 */
// something
public static boolean leftAutoLineExchangePath ()
{
    // prints the left auto switch state
    // System.out.println("LeftExchangeAuto" + leftExchangeAuto);
    SmartDashboard.putString("leftAutolineExchange state : ",
            leftExchangeAuto.toString());
    // left auto switch statement
    switch (leftExchangeAuto)
        {
        case PATH_INIT:
            // initializes everything necessary
            Hardware.drive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            leftExchangeAuto = leftExchangeState.DRIVE_ACROSS_AUTOLINE;

            break;

        case DRIVE_ACROSS_AUTOLINE:
            // drives the necessary distance across the autoline then
            // changes the state to DRIVE_BACK_ACROSS_AUTOLINE
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE - Hardware.drive
                            .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                {
                leftExchangeAuto = leftExchangeState.BRAKE_B4_DRIVE_BACK_ACROSS_AUTOLINE;
                }
            break;

        case BRAKE_B4_DRIVE_BACK_ACROSS_AUTOLINE:
            // brakes after driving
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                leftExchangeAuto = leftExchangeState.TIMER_DELAY_ONE;
                }
            break;

        case TIMER_DELAY_ONE:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                leftExchangeAuto = leftExchangeState.DRIVE_BACK_ACROSS_AUTOLINE;
                }
            break;

        case DRIVE_BACK_ACROSS_AUTOLINE:
            // drives backwards across the autoline and sets the state
            // to TURN_90_DEGREES_RIGHT
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_BACK_ACROSS_AUTOLINE - Hardware.drive
                            .getBrakeStoppingDistance(),
                    -DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                {
                leftExchangeAuto = leftExchangeState.BRAKE_B4_TURN;
                Hardware.drive.resetAccelerate();
                }
            break;

        case BRAKE_B4_TURN:
            // Brake after driving forwards and backwards
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                leftExchangeAuto = leftExchangeState.TIMER_DELAY_TWO;
                }
            break;

        case TIMER_DELAY_TWO:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                leftExchangeAuto = leftExchangeState.TURN_90_DEGREES_RIGHT;
                }
            break;

        case TURN_90_DEGREES_RIGHT:
            // turns 90 degrees towards the exchange and sets the state
            // to DRIVE_TO_EXCHANGE
            if (Hardware.drive.turnDegrees2Stage(
                    LEFT_SIDE_TURN_TOWARDS_EXCHANGE,
                    TURN_SPEED) == true)
                {
                leftExchangeAuto = leftExchangeState.BRAKE_AFTER_TURN;
                }
            break;

        case BRAKE_AFTER_TURN:
            // Brake after driving forwards and backwards
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                {
                // System.out.println("LeftFront: "
                // + Hardware.leftFrontDriveEncoder.getDistance());
                // System.out.println(
                // "RightFront: " + Hardware.rightFrontDriveEncoder
                // .getDistance());
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                leftExchangeAuto = leftExchangeState.TIMER_DELAY_THREE;
                }
            break;

        case TIMER_DELAY_THREE:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                leftExchangeAuto = leftExchangeState.DRIVE_TO_EXCHANGE;
                }
            break;

        case DRIVE_TO_EXCHANGE:
            // drives distance to the exchange and sets state to DONE
            if (Hardware.drive.driveStraightInches(
                    LEFT_DISTANCE_TO_EXCHANGE - Hardware.drive
                            .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                leftExchangeAuto = leftExchangeState.BRAKE_B4_DEPLOY;
            break;

        case BRAKE_B4_DEPLOY:
            // brake before deploying
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                leftExchangeAuto = leftExchangeState.DEPLOY;
                }
            break;

        case TIMER_DELAY_FOUR:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                leftExchangeAuto = leftExchangeState.TURN_90_DEGREES_RIGHT;
                }
            break;

        case DEPLOY:
            // deploys the cube intake DONE
            Hardware.cubeManipulator.deployCubeIntake(false);
            leftExchangeAuto = leftExchangeState.DONE;
            break;
        default:
            System.out.println(
                    "REACHED THE DEFAULT CASE OF LEFT EXCHANGE");
        case DONE:
            // robot stops and does nothing until teleop; returns true
            // so the main autonomous state machine knows this is done
            Hardware.transmission.stop();
            return true;
        }

    return false;
}

public static rightExchangeState rightExchangeAuto = rightExchangeState.DRIVE_ACROSS_AUTOLINE;

/**
 * Possible states for right exchange autonomous
 * 
 * @author Ashley Espeland
 *
 */
public static enum rightExchangeState
    {
PATH_INIT, DRIVE_ACROSS_AUTOLINE, BRAKE_B4_DRIVE_BACK_ACROSS_AUTOLINE, TIMER_DELAY_ONE, DRIVE_BACK_ACROSS_AUTOLINE, BRAKE_AFTER_STRAIGHT, TIMER_DELAY_TWO, TURN_90_DEGREES_LEFT, BRAKE_AFTER_TURN, TIMER_DELAY_THREE, DRIVE_TO_EXCHANGE, BRAKE_B4_DEPLOY, TIMER_DELAY_FOUR, DEPLOY, DONE
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
    SmartDashboard.putString("rightAutolineExchange state : ",
            rightExchangeAuto.toString());
    switch (rightExchangeAuto)
        {
        case PATH_INIT:
            // initializes everything necessary
            Hardware.drive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            rightExchangeAuto = rightExchangeState.DRIVE_ACROSS_AUTOLINE;

            break;
        case DRIVE_ACROSS_AUTOLINE:
            // drives the necessary distance across the autoline then
            // changes the state to DRIVE_BACK_ACROSS_AUTOLINE
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE, DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                {
                rightExchangeAuto = rightExchangeState.BRAKE_B4_DRIVE_BACK_ACROSS_AUTOLINE;
                Hardware.drive.resetAccelerate();
                }
            break;
        case BRAKE_B4_DRIVE_BACK_ACROSS_AUTOLINE:
            // brakes after driving
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                rightExchangeAuto = rightExchangeState.TIMER_DELAY_ONE;
                }
            break;

        case TIMER_DELAY_ONE:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                rightExchangeAuto = rightExchangeState.DRIVE_BACK_ACROSS_AUTOLINE;
                }
            break;

        case DRIVE_BACK_ACROSS_AUTOLINE:
            // drives backwards across the autoline and sets the state
            // to TURN_90_DEGREES_LEFT
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_BACK_ACROSS_AUTOLINE - Hardware.drive
                            .getBrakeStoppingDistance(),
                    -DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                rightExchangeAuto = rightExchangeState.BRAKE_AFTER_STRAIGHT;
            break;

        case BRAKE_AFTER_STRAIGHT:
            // Brake after driving forwards and backwards
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                rightExchangeAuto = rightExchangeState.TIMER_DELAY_TWO;
                }
            break;

        case TIMER_DELAY_TWO:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                rightExchangeAuto = rightExchangeState.TURN_90_DEGREES_LEFT;
                ;
                }
            break;

        case TURN_90_DEGREES_LEFT:
            // turns 90 degrees towards the exchange and sets the state
            // to DRIVE_TO_EXCHANGE
            if (Hardware.drive.turnDegrees2Stage(
                    RIGHT_SIDE_TURN_TOWARDS_EXCHANGE,
                    TURN_SPEED) == true)
                {
                rightExchangeAuto = rightExchangeState.BRAKE_AFTER_TURN;
                }
            break;

        case BRAKE_AFTER_TURN:
            // Brake after driving forwards and backwards
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                {
                // System.out.println("LeftFront: "
                // + Hardware.leftFrontDriveEncoder.getDistance());
                // System.out.println(
                // "RightFront: " + Hardware.rightFrontDriveEncoder
                // .getDistance());
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                rightExchangeAuto = rightExchangeState.TIMER_DELAY_THREE;
                }
            break;

        case TIMER_DELAY_THREE:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                rightExchangeAuto = rightExchangeState.DRIVE_TO_EXCHANGE;
                }
            break;

        case DRIVE_TO_EXCHANGE:
            // drives distance to the exchange and sets state to DONE
            if (Hardware.drive.driveStraightInches(
                    RIGHT_DISTANCE_TO_EXCHANGE, DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                {
                rightExchangeAuto = rightExchangeState.BRAKE_B4_DEPLOY;
                }
            break;

        case BRAKE_B4_DEPLOY:
            // brake before deploying
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                delayForBrakeTimer.reset();
                delayForBrakeTimer.start();
                rightExchangeAuto = rightExchangeState.TIMER_DELAY_FOUR;
                }
            break;

        case TIMER_DELAY_FOUR:
            // wait for the timer to be greater than the DELAY_FOR_BRAKE_TIME
            if (delayForBrakeTimer.get() > DELAY_FOR_BRAKE_TIME)
                {
                delayForBrakeTimer.stop();
                rightExchangeAuto = rightExchangeState.DEPLOY;
                }
            break;

        case DEPLOY:
            // deploys the cube intake DONE
            Hardware.cubeManipulator.deployCubeIntake(false);
            rightExchangeAuto = rightExchangeState.DONE;
            break;

        default:
            System.out.println(
                    "REACHED THE DEFAULT CASE OF RIGHT EXCHANGE");
        case DONE:
            // robot stops and does nothing until teleop; returns true
            // so the main autonomous state machine knows this is done
            Hardware.transmission.stop();
            return true;

        }

    return false;
}

public static centerState visionAuto = centerState.CENTER_INIT;

/**
 * Left Plan: Drive ten inches, brake, turn 90 degrees to left, drive 53 inches,
 * turn 90 degrees to right, drive 54 inches with camera, drive 24 inches with
 * ultrasonic
 * 
 * @return
 *         Whether or not the robot has finished the action
 */
public static boolean centerSwitchPath ()
{
    // System.out.println("Vision Auto state: " + visionAuto);
    // SMARTDASHBOARD.PUTSTRING("VISION States", visionAuto.toString());
    switch (visionAuto)
        {
        case CENTER_INIT:
            Hardware.drive.setDefaultAcceleration(CENTER_ACCEL);
            // start deploying the intake mechanism; will keep running in
            // background
            Hardware.cubeManipulator.deployCubeIntake(false);
            Hardware.cubeManipulator
                    .setLiftPosition(SWITCH_LIFT_HEIGHT, .7);
            Hardware.tempRelay.set(true);
            visionAuto = centerState.DRIVE_FOUR_INCHES;
            break;
        case DRIVE_FOUR_INCHES:
            // drive 4 inches to make the turn and sets state to BRAKE_1
            // -Hardware.autoDrive.getBrakeStoppingDistance()
            if (Hardware.drive.driveStraightInches(4,
                    MID_DRIVE_SPEED, 0, true) == true)
                {
                visionAuto = centerState.GRAB_DATA;// Bypass the first brake
                }
            break;
        case BRAKE_1:
            // brakes!!! and sets state to GRAB_DATA
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                visionAuto = centerState.GRAB_DATA;
                }
            break;
        case GRAB_DATA:
            // know where to go and sets state to the appropriate turn state
            // (whichever side is our side of the switch)
            SmartDashboard.putString("Switch data",
                    GameDataType.SWITCH.toString());
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                visionAuto = centerState.TURN_TOWARDS_LEFT_SIDE;
                }
            else if (grabData(GameDataType.SWITCH) == Position.RIGHT)
                {
                visionAuto = centerState.TURN_TOWARDS_RIGHT_SIDE;
                }
            else
                {
                visionAuto = centerState.DONE;
                }
            break;
        case TURN_TOWARDS_LEFT_SIDE:
            // Turn x degrees to the left, if the switch is on the left
            if (Hardware.drive.pivotTurnDegrees(-80,
                    MID_DRIVE_SPEED, true) == true)// Changed
            // from
            // 2stageturn
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_LEFT;// Bypass
                                                                       // brake
                }
            break;
        case TURN_TOWARDS_RIGHT_SIDE:
            // Turn x degrees to the right, if the switch is on the right
            if (Hardware.drive.pivotTurnDegrees(80,
                    MID_DRIVE_SPEED, true) == true)// Changed
            // from
            // 2stageTurn
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_RIGHT;// bypass
                                                                        // brake
                }
            break;
        case BRAKE_2_L:
            // brake switch is on left
            // sets state to DRIVE_STRAIGHT_TO_SWITCH_LEFT
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_LEFT;
                }
            break;
        case BRAKE_2_R:
            // brake switch is on right
            // sets state to DRIVE_STRAIGHT_TO_SWITCH_RIGHT
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_RIGHT;
                }
            break;
        case DRIVE_STRAIGHT_TO_SWITCH_LEFT:
            // drive straight, switch is on the left then brakes
            // sets state to TURN_AGAIN_LEFT
            if (Hardware.drive.driveStraightInches(
                    DRIVE_NO_CAMERA_LEFT, DRIVE_SPEED, 0, true) == true)
                {
                visionAuto = centerState.TURN_AGAIN_LEFT;
                }
            break;
        case BRAKE_3_L:
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                visionAuto = centerState.TURN_AGAIN_LEFT;
                }
            break;
        case DRIVE_STRAIGHT_TO_SWITCH_RIGHT:
            // drive straight, switch is on the right then brakes
            // sets state to TURN_AGAIN_RIGHT
            if (Hardware.drive.driveStraightInches(
                    DRIVE_NO_CAMERA_RIGHT, DRIVE_SPEED, 0,
                    true) == true)
                {
                // visionAuto = centerState.DONE;
                visionAuto = centerState.TURN_AGAIN_RIGHT;
                }
            break;
        case BRAKE_3_R:
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                visionAuto = centerState.TURN_AGAIN_RIGHT;
                }
            break;
        case TURN_AGAIN_RIGHT:
            // turn 90 to the left and sets the state to DRIVE_WITH_CAMERA this
            // is the right side auto
            if (Hardware.drive.pivotTurnDegrees(-80,
                    MID_DRIVE_SPEED, true) == true)
                {
                visionAuto = centerState.DRIVE_WITH_CAMERA;
                }
            break;
        case TURN_AGAIN_LEFT:
            // turns 90 to the right then brakes and sets the state to
            // DRIVE_WITH_CAMERA this is the left side auto
            if (Hardware.drive.pivotTurnDegrees(65,
                    MID_DRIVE_SPEED, true) == true)
                {
                visionAuto = centerState.DRIVE_WITH_CAMERA;
                }
            break;
        case BRAKE_AFTER_LEFT_TURN_2:
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                {
                // if we are using the camera, go into the drive with camera
                // state
                if (usingAutoCamera == true)
                    {
                    visionAuto = centerState.DRIVE_WITH_CAMERA;
                    }
                else
                    {
                    // if we are using the camera, drive to the switch with no
                    // camera
                    visionAuto = centerState.DRIVE_STRAIGHT_NO_CAMERA;
                    }
                }
            break;
        case BRAKE_AFTER_RIGHT_TURN_2:
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                {
                // if we are using the camera, go into the drive with camera
                // state
                if (usingAutoCamera == true)
                    {
                    visionAuto = centerState.DRIVE_WITH_CAMERA;
                    }
                else
                    {
                    // if we are using the camera, drive to the switch with no
                    // camera
                    if (!firstTimeUltrasonic)
                        {
                        System.out.println("We are "
                                + Hardware.frontUltraSonic
                                        .getDistanceFromNearestBumper()
                                + " inches away from the nearest object");
                        firstTimeUltrasonic = true;
                        }

                    visionAuto = centerState.DRIVE_STRAIGHT_NO_CAMERA;
                    }
                }
            break;
        case DRIVE_WITH_CAMERA:
            // drives to the switch based on the camera
            // sets state to LIFT
            Hardware.tempRelay.set(true);
            if (Hardware.driveWithCamera
                    .driveToSwitch(AUTO_SPEED_VISION) == true)
                {
                // Turn off the ringlight camera
                Hardware.tempRelay.set(false);
                // Hardware.transmission.stop();
                visionAuto = centerState.MAKE_DEPOSIT;
                }
            break;
        case DRIVE_STRAIGHT_NO_CAMERA:
            Hardware.drive.driveStraight(AUTO_SPEED_VISION, 0, true);
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() <= 15)
                {
                Hardware.transmission.stop();
                visionAuto = centerState.LIFT;
                }
            break;
        case LIFT:// DO NOT USE!(too bad I'm using it) it sets the lift position
                  // in this auto's init.
            // moves the forklift to the scale height and holds it there
            // sets state to MAKE_DEPOSIT
            Hardware.transmission.stop();
            if (Hardware.cubeManipulator.setLiftPosition(
                    CubeManipulator.SWITCH_HEIGHT,
                    FORKLIFT_SPEED) == true)
                {
                visionAuto = centerState.MAKE_DEPOSIT;
                }
            break;
        case MAKE_DEPOSIT:
            // deposits cube on switch and sets state to DONE
            Hardware.transmission.stop();
            if (Hardware.cubeManipulator.pushOutCubeAuto(.3) == true)
                {
                visionAuto = centerState.DONE;
                }
            break;
        default: // prints that we reached the default case, then falls through
                 // to DONE
            System.out.println(
                    "REACHED THE DEFAULT CASE FOR centerSwitchPath()");
        case DONE:
            // stops robot the robot, and return true so the main auto
            // switch machine knows this path is done
            Hardware.drive.brake(BrakeType.AFTER_DRIVE);
            Hardware.transmission.stop();
            return true;
        }
    return false;
}

public static boolean usingAutoCamera = true;

public static boolean firstTimeUltrasonic = false;

/**
 * Possible states for center vision autonomous
 * 
 * @author Becky Button
 *
 */
public static enum centerState
    {

CENTER_INIT, DRIVE_FOUR_INCHES, BRAKE_1, GRAB_DATA,
/**
 * Left side auto, turns 90 degrees to the left
 */
TURN_TOWARDS_LEFT_SIDE,
/**
 * Right side auto, turns 90 degrees to the right
 */
TURN_TOWARDS_RIGHT_SIDE,

/**
 * Left side auto, brakes
 */
BRAKE_2_L,
/**
 * Right side auto, brakes
 */
BRAKE_2_R,
/**
 * Left side auto, Drive the left side distance of 73 inches
 */
DRIVE_STRAIGHT_TO_SWITCH_LEFT,
/**
 * Right side auto, Drive the right side distance of (yet to be calculated)
 */
DRIVE_STRAIGHT_TO_SWITCH_RIGHT,
/**
 * Brakes after DRIVE_STRAIGHT_TO_SWITCH_LEFT
 */
BRAKE_3_L,
/**
 * Brakes after DRIVE_STRAIGHT_TO_SWITCH_RIGHT
 */
BRAKE_3_R,
/**
 * Left side auto, turn 90 degrees to the right
 */
TURN_AGAIN_LEFT,
/**
 * Right side auto, turn 90 degrees to the left
 */
BRAKE_AFTER_LEFT_TURN_2, BRAKE_AFTER_RIGHT_TURN_2, TURN_AGAIN_RIGHT,
/**
 * Drives with camera, then stops see driveToSwitch() in drive.
 */
DRIVE_WITH_CAMERA, DRIVE_STRAIGHT_NO_CAMERA,
/**
 * Raises the lift SWITCH_LIFT_HEIGHT
 */
LIFT,
/**
 * Spits the power cube into the switch
 */
MAKE_DEPOSIT,
/**
 * Sets everything to zero;
 */
DONE
    }

/**
 * Delivers the cube to the switch if it's on our side... If not, then
 * setup to either the left or right scale, all based on game data from the
 * driver station.
 * 
 * @param robotPosition
 *            denote left or right
 * @return
 *         Whether or not the robot has finished the path
 */
public static boolean switchOrScalePath (Position robotPosition)
{
    // prints the current state for this autonomous path
    // System.out.println("Current State: " + currentSwitchOrScaleState);

    switch (currentSwitchOrScaleState)
        {
        case PATH_INIT:
            // starts deploying the cube intake and moves on to the next state
            // does not wait for the intake to finish deploying before moving
            // onto the next state
            Hardware.cubeManipulator.deployCubeIntake(false);
            Hardware.cubeManipulator
                    .setLiftPosition(CubeManipulator.SWITCH_HEIGHT);

            // Scale is on our side? Go to it EVEN IF the switch is also on our
            // side.
            if (grabData(GameDataType.SCALE) == robotPosition)
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE2;
            else
                // // Else go to switch
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE1;
            break;
        case DRIVE1:
            // FIRST driveInches: drive forward to switch

            // if we've finished driving this segment
            if (Hardware.drive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[0]
                            - Hardware.drive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                {
                // turn to switch
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN1;
                }
            break;
        case BRAKE_DRIVE1:
            // Brake before turning towards the switch
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN1;
            break;
        case TURN1:
            // Turn towards the switch
            if (robotPosition == Position.RIGHT)
                {
                // We are on the Right side? turn left.
                if (Hardware.drive.turnDegrees2Stage(-90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_TURN1;
                }
            else
                {
                // We are on the Left side? turn right.
                if (Hardware.drive.turnDegrees2Stage(90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_TURN1;
                }
            break;
        case BRAKE_TURN1:
            // Brake after turning, and before going to the switch with
            // ultrasonic
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_WITH_ULTRSNC;
            break;
        case DRIVE_WITH_ULTRSNC:
            Hardware.drive.driveStraight(DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true);
            // Drive to the switch until the ultrasonic tells us to stop
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() < MIN_ULTRSNC_DISTANCE)
                currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_ULTRSNC;

            break;
        case BRAKE_ULTRSNC:
            // Brake after driving using the ultrasonic
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                Hardware.autoTimer.reset();
                Hardware.autoTimer.start();
                currentSwitchOrScaleState = SwitchOrScaleStates.EJECT_CUBE_SWITCH;
                }
            break;
        case EJECT_CUBE_SWITCH:
            // Eject the cube onto the switch platform.
            if (Hardware.cubeManipulator.pushOutCubeAuto())
                {
                currentSwitchOrScaleState = SwitchOrScaleStates.FINISH;
                }

            break;
        // ================END SWITCH PATH================

        // ================BEGIN SCALE PATH===============
        case DRIVE2:
            // Drive FAST to the middle of the platform zone
            if (Hardware.drive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[1], FAST_DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true))

                {
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE3;
                // Reset encoders for safety in next state
                Hardware.drive.resetEncoders();
                }
            break;
        case DRIVE3:
            // drive SLOW to the tape in front of the scale
            // Have we reached the line yet?
            if (Hardware.redLight.isOn() == true)
                {
                Hardware.cubeManipulator
                        .setLiftPosition(CubeManipulator.SCALE_HEIGHT);
                currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE3;
                }
            // Safety encoder reading: Make sure we don't get a penalty for
            // going into the opposition zone
            else if (Hardware.drive.isAnyEncoderLargerThan(130))
                currentSwitchOrScaleState = SwitchOrScaleStates.FINISH;
            else
                // Keep driving forwards
                Hardware.drive.driveStraight(SLOW_DRIVE_SPEED,
                        0, true);

            break;
        case BRAKE_DRIVE3:
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE))
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN_TO_SCALE;
            break;
        case TURN_TO_SCALE:
            // We are on the left side? turn right.
            if (robotPosition == Position.LEFT)
                {
                if (Hardware.drive.turnDegrees2Stage(
                        SWITCH_OR_SCALE_SCALE_ANGLE, TURN_SPEED))
                    currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_TO_SCALE;

                }
            // We are on the right side? turn left.
            else if (robotPosition == Position.RIGHT)
                {
                if (Hardware.drive.turnDegrees2Stage(
                        -SWITCH_OR_SCALE_SCALE_ANGLE, TURN_SPEED))
                    currentSwitchOrScaleState = SwitchOrScaleStates.RAISE_ARM;
                }
            else
                // We dont know what the heck is going on? just end it man.
                currentSwitchOrScaleState = SwitchOrScaleStates.RAISE_ARM;

            break;
        case DRIVE_TO_SCALE:
            // Drive a little distance forwards before raising the arm, after
            // turning.
            if (Hardware.drive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[2], SLOW_DRIVE_SPEED,
                    0, true))
                currentSwitchOrScaleState = SwitchOrScaleStates.RAISE_ARM;
            break;
        case BACK_UP_FROM_SCALE:
            // Drive away from the scale if we are too close
            Hardware.drive.driveStraight(-SLOW_DRIVE_SPEED, 0, true);
            currentSwitchOrScaleState = SwitchOrScaleStates.RAISE_ARM;
            break;
        case RAISE_ARM:
            // If the sensor reads we are under the scale, don't do that. Back
            // it up m8
            if (Hardware.armIR.isOn() == true)
                {
                Hardware.cubeManipulator.stopForklift();
                currentSwitchOrScaleState = SwitchOrScaleStates.BACK_UP_FROM_SCALE;
                break;
                }
            // If we are not under the scale, then wait until we set the lift
            // position to be able to score on the scale.
            else if (Hardware.cubeManipulator
                    .setLiftPosition(CubeManipulator.SCALE_HEIGHT))
                {
                Hardware.transmission.stop();
                currentSwitchOrScaleState = SwitchOrScaleStates.ANGLE_DEPLOY;
                }
            else
                // We will keep running backwards if we don't stop the
                // transmission
                Hardware.transmission.stop();

            break;
        case ANGLE_DEPLOY:
            // Set the deploy to a 45 degree angle to launch the cube
            if (Hardware.cubeManipulator.angleDeployForScale() == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.EJECT_CUBE_SCALE;
            break;
        case EJECT_CUBE_SCALE:
            // Push out the cube, keeping it at a 45 degree angle.
            Hardware.cubeManipulator.angleDeployForScale();
            if (Hardware.cubeManipulator.pushOutCubeAuto(.6) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_4;
            break;
        case DRIVE_4:
            // SLOWLY drive away from the scale with arm up to avoid getting a
            // penalty
            if (Hardware.drive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[3], SLOW_DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true))
                currentSwitchOrScaleState = SwitchOrScaleStates.LOWER_ARM;
            break;
        case LOWER_ARM:
            // Bring the arm back down for teleop
            if (Hardware.cubeManipulator.setLiftPosition(0) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN2;
            break;
        case TURN2:
            // Turn back towards the switch based on which side we are on, to
            // set up for more points
            if (robotPosition == Position.LEFT)
                {
                if (Hardware.drive.turnDegrees2Stage(90,
                        TURN_SPEED))
                    currentSwitchOrScaleState = SwitchOrScaleStates.FINISH;
                }
            else
                {
                if (Hardware.drive.turnDegrees2Stage(-90,
                        TURN_SPEED))
                    currentSwitchOrScaleState = SwitchOrScaleStates.FINISH;
                }
            break;
        default: // prints that we reached the default, then falls through to
                 // FINISH--
            System.out.println(
                    "REACHED THE DEFAULT CASE for switchOrScalePath()");
        case FINISH:
            // if we don't know what's going on (default case) or we're
            // finished, stop driving and return true so the main autonomous
            // state can continue
            Hardware.transmission.stop();
            Hardware.cubeManipulator.stopEverything();
            return true;
        }
    return false;
}

// variable that controls the states for the Switch or Scale autonomous path
private static SwitchOrScaleStates currentSwitchOrScaleState = SwitchOrScaleStates.PATH_INIT;

/**
 * Enum for the possible states for the Switch or Scale autonomous path
 */
private static enum SwitchOrScaleStates
    {
PATH_INIT, DRIVE1, BRAKE_DRIVE1, TURN1, BRAKE_TURN1, DRIVE_WITH_ULTRSNC, BRAKE_ULTRSNC, EJECT_CUBE_SWITCH, DRIVE2, DRIVE3, BRAKE_DRIVE3, TURN_TO_SCALE, DRIVE_TO_SCALE, RAISE_ARM, ANGLE_DEPLOY, BACK_UP_FROM_SCALE, EJECT_CUBE_SCALE, DRIVE_4, LOWER_ARM, TURN2, FINISH
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
    // prints the current state for this autonomous path
    System.out.println("Current State: " + currentOffsetSwitchState);

    // switch statement/ state machine for this autonomous path
    switch (currentOffsetSwitchState)
        {
        case PATH_INIT:
            Hardware.drive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            currentOffsetSwitchState = OffsetSwitchPath.DRIVE1;
            // tell the cube intake mechanism to deploy
            Hardware.cubeManipulator.deployCubeIntake(false);
            break;

        case DRIVE1:
            // Drive forward a little to allow turning.
            if (Hardware.drive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[0]
                            - Hardware.drive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE1;
            break;
        case BRAKE_DRIVE1:
            // Brake after driving forwards
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                currentOffsetSwitchState = OffsetSwitchPath.TURN1;

            break;
        case TURN1:
            // Turn either left or right based on the game data provided
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                // If the switch is on the left side, then turn left
                if (Hardware.drive.turnDegrees2Stage(-90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN1;
                }
            else
                {
                // If the switch is on the right side, then turn right
                if (Hardware.drive.turnDegrees2Stage(90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN1;
                }
            break;
        case BRAKE_TURN1:
            // Brake after the first turn
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                {
                if (grabData(GameDataType.SWITCH) == Position.LEFT)
                    // If the switch is on the left side, then drive the left
                    // path
                    currentOffsetSwitchState = OffsetSwitchPath.DRIVE2L;
                else
                    // If not, drive the right path.
                    currentOffsetSwitchState = OffsetSwitchPath.DRIVE2R;
                }
            break;
        case DRIVE2L:
            // Drive if during turn 1, we turned left
            if (Hardware.drive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[1]
                            - Hardware.drive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE2;
            break;
        case DRIVE2R:
            // Drive if during turn 1, we turned right.
            if (Hardware.drive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[2]
                            - Hardware.drive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE2;
            break;
        case BRAKE_DRIVE2:
            // Brake after the 2nd drive function
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                currentOffsetSwitchState = OffsetSwitchPath.TURN2;
            break;

        case TURN2:
            // Turn after driving parallel to the driver station wall
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                // switch is on the left side? turn right
                if (Hardware.drive.turnDegrees2Stage(90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN2;
                }
            else
                {
                // Switch is on the right side? turn left
                if (Hardware.drive.turnDegrees2Stage(-90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN2;
                }
            break;
        case BRAKE_TURN2:
            // Brake after turning towards the opposing alliance
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                currentOffsetSwitchState = OffsetSwitchPath.RAISE_ARM;
            break;

        case RAISE_ARM:

            // tell the forklift to start raising up so we can drop off
            // the cube on the switch later
            Hardware.cubeManipulator.setLiftPosition(SWITCH_LIFT_HEIGHT,
                    FORKLIFT_SPEED);

            // Go the correct distance based on which side we are going to.
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                currentOffsetSwitchState = OffsetSwitchPath.DRIVE3L;
            else if (grabData(GameDataType.SWITCH) == Position.RIGHT)
                currentOffsetSwitchState = OffsetSwitchPath.DRIVE3R;
            else
                currentOffsetSwitchState = OffsetSwitchPath.FINISH;

            break;

        case DRIVE3L:
            // Drive to the middle of the end of the switch on the left path.
            if (Hardware.drive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[4]
                            - Hardware.drive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE3;
            break;
        case DRIVE3R:
            // Drive to the middle of the end of the switch on the right path.
            if (Hardware.drive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[3]
                            - Hardware.drive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE3;
            break;
        case BRAKE_DRIVE3:
            // Brake after driving to the center of the switch
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                currentOffsetSwitchState = OffsetSwitchPath.TURN3;
            break;
        case TURN3:
            // Turn towards the switch
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                // Switch is on the left side? turn right.
                if (Hardware.drive.turnDegrees2Stage(90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN3;
                }
            else
                {
                // Switch is on the right side? turn left.
                if (Hardware.drive.turnDegrees2Stage(-90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN3;
                }
            break;
        case BRAKE_TURN3:
            // Brake after turning towards the switch
            if (Hardware.drive.brake(BrakeType.AFTER_TURN) == true)
                currentOffsetSwitchState = OffsetSwitchPath.DRIVE_WITH_ULTRSNC;
            break;

        case DRIVE_WITH_ULTRSNC:
            // Drive towards the switch using the ultrasonic
            Hardware.drive.driveStraight(DRIVE_SPEED,
                    DRIVE_STRAIGHT_ACCELERATION_TIME, true);
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() < MIN_ULTRSNC_DISTANCE)
                {
                Hardware.transmission.stop();
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_B4_EJECT;
                }
            break;

        case BRAKE_B4_EJECT:
            // Brake after driving with the ultrasonic, before we eject the cube
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                currentOffsetSwitchState = OffsetSwitchPath.EJECT;
                }
            break;
        case EJECT:
            // Eject the cube we currently have into the switch :)
            if (Hardware.cubeManipulator.pushOutCubeAuto() == true)
                {
                currentOffsetSwitchState = OffsetSwitchPath.FINISH;
                }
            break;
        default: // prints out a message to let the console know we got here,
                 // then falls through to FINISH
            System.out.println(
                    "REACHED THE DEFAULT CASE for offsetSwitchPath()");
        case FINISH:
            // Finished with the offset path; stops motors and returns true
            // so the main auto state machine knows we're finished
            Hardware.transmission.stop();
            Hardware.cubeIntakeMotor.stopMotor();
            return true;
        }
    return false;
}

// variable that controls the states for the offsetSwitchPath autonomous
private static OffsetSwitchPath currentOffsetSwitchState = OffsetSwitchPath.PATH_INIT;

/**
 * Enum for the possible states of the offsetSwitchPath autonomous
 */
private enum OffsetSwitchPath
    {
PATH_INIT, DEPLOY_INTAKE, DRIVE1, BRAKE_DRIVE1, TURN1, BRAKE_TURN1, DRIVE2L, DRIVE2R, BRAKE_DRIVE2, TURN2, BRAKE_TURN2, DRIVE3L, DRIVE3R, RAISE_ARM_AND_DRIVE3_2, BRAKE_DRIVE3, TURN3, BRAKE_TURN3, RAISE_ARM, DRIVE_WITH_ULTRSNC, BRAKE_B4_EJECT, EJECT, FINISH
    }

/*
 * ================================ Constants
 * ================================
 */

// DRIVING
private static final double AUTO_TESTING_SCALAR = 1.0; // percent

private static final double DRIVE_STRAIGHT_ACCELERATION_TIME = .6; // seconds

private static final double DRIVE_SPEED = .5; // percent

private static final double FAST_DRIVE_SPEED = .55;

private static final double MID_DRIVE_SPEED = .3;

private static final double SLOW_DRIVE_SPEED = .15;

private static final double TURN_SPEED = .25; // percent

// ==========

// OPERATING
private static final double FORKLIFT_SPEED = .5;

private static final double INTAKE_SPEED = 1;

private static final double INTAKE_EJECT_TIME = 1;// Seconds
// ==========

// LIFT HEIGHT
private static final int SWITCH_LIFT_HEIGHT = 38;// Inches

private static final int SCALE_LIFT_HEIGHT = 78;// Inches
// ==========

// ULTRASONIC
private static final int MIN_ULTRSNC_DISTANCE = 6;// Inches
// ==========

// INIT

// DELAY

// CHOOSE_PATH

// AUTOLINE
private final static int DISTANCE_TO_CROSS_AUTOLINE = 120;

// AUTOLINE_SCALE
private final static int DISTANCE_TO_CROSS_AUTOLINE_AND_GO_TO_SCALE = 207;

// AUTOLINE_EXCHANGE
// distance required to drive back across the autoline before turning to go
// towards the exchange

public static final Timer delayForBrakeTimer = new Timer();

private final static double DELAY_FOR_BRAKE_TIME = .2;

private final static int DISTANCE_BACK_ACROSS_AUTOLINE = 100;

private final static int LEFT_DISTANCE_TO_EXCHANGE = 58;

private final static int LEFT_SIDE_TURN_TOWARDS_EXCHANGE = 90;

private final static int RIGHT_SIDE_TURN_TOWARDS_EXCHANGE = -90;

private final static int RIGHT_DISTANCE_TO_EXCHANGE = 130;

// CENTER_SWITCH
private final static double DRIVE_NO_CAMERA_LEFT = 26.5;// Inches

private final static double DRIVE_NO_CAMERA_RIGHT = 16.3;// inches

private final static double CENTER_ACCEL = .6;

// TODO change for actual auto speed
private final static double AUTO_SPEED_VISION = .25;

// SWITCH_OR_SCALE
// array for storing the different driving distances in SWITH_OR_SCALE
private static final int[] SWITCH_OR_SCALE_DRIVE_DISTANCE = new int[]
// distance to be perpendicular to the switch
    {(int) (AUTO_TESTING_SCALAR * 126),
            // distance to drive to the middle of the platform zone
            (int) (AUTO_TESTING_SCALAR * 150),
            (int) (AUTO_TESTING_SCALAR * 4),
            (int) (AUTO_TESTING_SCALAR) * -6};

private static final int SWITCH_OR_SCALE_SCALE_ANGLE = 53;// degrees

// OFFSET_SWITCH
// array for storing the different driving distances used in OFFSET_SWITCH
// values in the array (in order) are for DRIVE1, DRIVE2L, DRIVE2R, DRIVE3R,
// DRIVE3L
private static final int[] OFFSET_SWITCH_DRIVE_DISTANCES = new int[]
    {(int) (AUTO_TESTING_SCALAR * 12),
            (int) (AUTO_TESTING_SCALAR * 180),
            (int) (AUTO_TESTING_SCALAR * 44),
            (int) (AUTO_TESTING_SCALAR * 127),
            (int) (AUTO_TESTING_SCALAR * 117)};

// FINISH

} // end class
