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
import org.usfirst.frc.team339.HardwareInterfaces.transmission.Drive.BrakeType;
import edu.wpi.first.wpilibj.DriverStation;

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
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.liftingEncoder.reset();
    Hardware.intakeDeployEncoder.reset();
    // Disable auto
    if (Hardware.disableAutonomousSwitch.isOn() == true)
        autoState = State.FINISH;

    Hardware.transmission.setForAutonomous();
    Hardware.autoDrive
            .setDefaultAcceleration(DRIVE_STRAIGHT_ACCELERATION_TIME);
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
             * States:
             * 0 = AUTOLINE
             * 1 = AUTOLINE THEN SCALE
             * 2 = AUTOLINE THEN EXCHANGE
             * 3 = CENTER SWITCH W/ VISION
             * 4 = SWITCH OR SCALE W/ GAME DATA
             * 5 = OFFSET SWITCH DROP OFF
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
                    if (Hardware.onNessie == true)
                        {
                        autoState = State.CENTER_SWITCH;
                        }
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
                    // use vision and gamedata to drive to the correct switch
                    // side and drop off the cube
                    autoState = State.CENTER_SWITCH;
                    break;
                case 4:
                    // if the robot starts on the right side for the switch,
                    // drop of the cube
                    if (Hardware.leftAutoSwitch.isOn() == true)
                        autoState = State.SWITCH_OR_SCALE_L;
                    else
                        autoState = State.SWITCH_OR_SCALE_R;
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
            Hardware.autoTimer.stop();
            Hardware.autoTimer.reset();
            break;

        default:
            // if something goes wrong, stop autonomous and go straight
            // to FINISH
            Hardware.transmission.stop();
            Hardware.autoTimer.stop();
            Hardware.autoTimer.reset();
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
    // gets the game data the driver station provides us with
    String gameData = DriverStation.getInstance()
            .getGameSpecificMessage();

    // Stay in this state if there are not 3 letters in the game data
    // provided
    if (gameData.length() < 3)
        return Position.NULL;
    // return null

    // sets the switch position based on the game data
    if (dataType == GameDataType.SWITCH
            && gameData.charAt(0) == 'L')
        {
        return Position.LEFT;
        }
    else if (dataType == GameDataType.SWITCH
            && gameData.charAt(0) == 'R')
        {
        return Position.RIGHT;
        }

    // sets the scale position based on the game data
    if (dataType == GameDataType.SCALE && gameData.charAt(1) == 'L')
        {
        return Position.LEFT;
        }
    else if (dataType == GameDataType.SCALE
            && gameData.charAt(1) == 'R')
        {
        return Position.RIGHT;
        }
    // basically default if something goes wrong
    return Position.NULL;
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

// TODO @ANE add in brake to your auto methods
public static boolean autolinePath ()
{
    System.out.println("autoline path state : " + currentAutolineState);

    // autoline switch statement
    switch (currentAutolineState)
        {
        // Initialize anything necessary
        case PATH_INIT:
            Hardware.autoDrive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            currentAutolineState = AutolinePathStates.DRIVE1;
            break;
        case DRIVE1:
            // Drive across the line
            if (Hardware.autoDrive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE - Hardware.autoDrive
                            .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentAutolineState = AutolinePathStates.BRAKE1;
            break;
        case BRAKE1:
            // Brake after driving across the line

            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentAutolineState = AutolinePathStates.DEPLOY;
            break;
        case DEPLOY:
            if (Hardware.cubeManipulator.deployCubeIntake())
                {
                currentAutolineState = AutolinePathStates.FINISH;
                }
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

// Enum for the states in the autolinePath and autolineScalePath autonomouses
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
            Hardware.autoDrive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            currentAutolineState = AutolinePathStates.DRIVE1;
            break;
        case DRIVE1:
            // Drive across the auto line, and a little farther to be closer to
            // the scale
            if (Hardware.autoDrive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE_AND_GO_TO_SCALE
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentAutolineState = AutolinePathStates.BRAKE1;
            break;
        case BRAKE1:
            // Brake after driving across the line
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
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
PATH_INIT, DRIVE_ACROSS_AUTOLINE, DRIVE_BACK_ACROSS_AUTOLINE, BRAKE_AFTER_STRAIGHT, TURN_90_DEGREES_RIGHT, BRAKE_AFTER_TURN, DRIVE_TO_EXCHANGE, DEPLOY, DONE
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
    System.out.println("LeftExchangeAuto" + leftExchangeAuto);

    // left auto switch statement
    switch (leftExchangeAuto)
        {
        case PATH_INIT:
            // initializes everything necessary
            Hardware.autoDrive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            leftExchangeAuto = leftExchangeState.DRIVE_ACROSS_AUTOLINE;

            break;

        case DRIVE_ACROSS_AUTOLINE:
            // drives the necessary distance across the autoline then
            // changes the state to DRIVE_BACK_ACROSS_AUTOLINE
            if (Hardware.autoDrive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE,
                    DRIVE_SPEED) == true)
                {
                leftExchangeAuto = leftExchangeState.DRIVE_BACK_ACROSS_AUTOLINE;
                }
            break;

        case DRIVE_BACK_ACROSS_AUTOLINE:
            // drives backwards across the autoline and sets the state
            // to TURN_90_DEGREES_RIGHT
            if (Hardware.autoDrive.driveStraightInches(
                    DISTANCE_BACK_ACROSS_AUTOLINE
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    -DRIVE_SPEED) == true)
                {
                leftExchangeAuto = leftExchangeState.BRAKE_AFTER_STRAIGHT;
                Hardware.autoDrive.resetAccelerate();
                }
            break;

        case BRAKE_AFTER_STRAIGHT:
            // Brake after driving forwards and backwards
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                leftExchangeAuto = leftExchangeState.TURN_90_DEGREES_RIGHT;
            break;

        case TURN_90_DEGREES_RIGHT:
            // turns 90 degrees towards the exchange and sets the state
            // to DRIVE_TO_EXCHANGE
            if (Hardware.autoDrive.turnDegrees(
                    LEFT_SIDE_TURN_TOWARDS_EXCHANGE,
                    TURN_SPEED) == true)
                {
                leftExchangeAuto = leftExchangeState.BRAKE_AFTER_TURN;
                }
            break;

        case BRAKE_AFTER_TURN:
            // Brake after driving forwards and backwards
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                {
                System.out.println("LeftFront: "
                        + Hardware.leftFrontDriveEncoder.getDistance());
                System.out.println(
                        "RightFront: " + Hardware.rightFrontDriveEncoder
                                .getDistance());
                leftExchangeAuto = leftExchangeState.DRIVE_TO_EXCHANGE;
                }
            break;

        case DRIVE_TO_EXCHANGE:
            // drives distance to the exchange and sets state to DONE
            if (Hardware.autoDrive.driveStraightInches(
                    LEFT_DISTANCE_TO_EXCHANGE,
                    DRIVE_SPEED) == true)
                leftExchangeAuto = leftExchangeState.DEPLOY;
            break;

        case DEPLOY:
            // deploys the cube intake DONE
            Hardware.cubeManipulator.deployCubeIntake();
            leftExchangeAuto = leftExchangeState.DONE;
            break;
        default:
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
PATH_INIT, DRIVE_ACROSS_AUTOLINE, DRIVE_BACK_ACROSS_AUTOLINE, BRAKE_AFTER_STRAIGHT, TURN_90_DEGREES_LEFT, BRAKE_AFTER_TURN, DRIVE_TO_EXCHANGE, DEPLOY, DONE
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
        case PATH_INIT:
            // initializes everything necessary
            Hardware.autoDrive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            rightExchangeAuto = rightExchangeState.DRIVE_ACROSS_AUTOLINE;

            break;
        case DRIVE_ACROSS_AUTOLINE:
            // drives the necessary distance across the autoline then
            // changes the state to DRIVE_BACK_ACROSS_AUTOLINE
            if (Hardware.autoDrive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE,
                    DRIVE_SPEED) == true)
                {
                rightExchangeAuto = rightExchangeState.DRIVE_BACK_ACROSS_AUTOLINE;
                Hardware.autoDrive.resetAccelerate();
                }
            break;

        case DRIVE_BACK_ACROSS_AUTOLINE:
            // drives backwards across the autoline and sets the state
            // to TURN_90_DEGREES_LEFT
            if (Hardware.autoDrive.driveStraightInches(
                    DISTANCE_BACK_ACROSS_AUTOLINE - Hardware.autoDrive
                            .getBrakeStoppingDistance(),
                    -DRIVE_SPEED) == true)
                rightExchangeAuto = rightExchangeState.BRAKE_AFTER_STRAIGHT;
            break;

        case BRAKE_AFTER_STRAIGHT:
            // Brake after driving forwards and backwards
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                rightExchangeAuto = rightExchangeState.TURN_90_DEGREES_LEFT;
            break;

        case TURN_90_DEGREES_LEFT:
            // turns 90 degrees towards the exchange and sets the state
            // to DRIVE_TO_EXCHANGE
            if (Hardware.autoDrive.turnDegrees(
                    RIGHT_SIDE_TURN_TOWARDS_EXCHANGE,
                    TURN_SPEED) == true)
                {
                rightExchangeAuto = rightExchangeState.BRAKE_AFTER_TURN;
                }
            break;

        case BRAKE_AFTER_TURN:
            // Brake after driving forwards and backwards
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                {
                System.out.println("LeftFront: "
                        + Hardware.leftFrontDriveEncoder.getDistance());
                System.out.println(
                        "RightFront: " + Hardware.rightFrontDriveEncoder
                                .getDistance());
                rightExchangeAuto = rightExchangeState.DRIVE_TO_EXCHANGE;
                }
            break;

        case DRIVE_TO_EXCHANGE:
            // drives distance to the exchange and sets state to DONE
            if (Hardware.autoDrive.driveStraightInches(
                    RIGHT_DISTANCE_TO_EXCHANGE,
                    DRIVE_SPEED) == true)
                {
                rightExchangeAuto = rightExchangeState.DEPLOY;
                }
            break;

        case DEPLOY:
            // deploys the cube intake DONE
            Hardware.cubeManipulator.deployCubeIntake();
            rightExchangeAuto = rightExchangeState.DONE;
            break;

        default:
        case DONE:
            // robot stops and does nothing until teleop; returns true
            // so the main autonomous state machine knows this is done
            Hardware.transmission.stop();
            return true;

        }

    return false;
}

/**
 * Left Plan: Drive ten inches, brake, turn 90 degrees to left, drive 73 inches,
 * turn 90 degrees to right, drive 54 inches with camera, drive 24 inches with
 * ultrasonic
 * 
 * @return
 *         Whether or not the robot has finished the action
 */
public static boolean centerSwitchPath ()
{
    // System.out.println("We are in the " + visionAuto + " state.");
    switch (visionAuto)
        {
        case CENTER_INIT:
            Hardware.autoDrive.setDefaultAcceleration(CENTER_ACCEL);
            visionAuto = centerState.DRIVE_TEN_INCHES;
            break;
        case DRIVE_TEN_INCHES:
            // drive 10 inches to make the turn and sets state to BRAKE_1
            // -Hardware.autoDrive.getBrakeStoppingDistance()
            if (Hardware.autoDrive.driveStraightInches(
                    10, AUTO_SPEED_VISION) == true)
                {
                visionAuto = centerState.BRAKE_1;
                }
            break;
        case BRAKE_1:
            // brakes!!! and sets state to GRAB_DATA
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                visionAuto = centerState.GRAB_DATA;
                }
            break;
        case GRAB_DATA:
            // know where to go and sets state to the appropriate turn state
            // (whichever side is our side of the switch)
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                visionAuto = centerState.TURN_TOWARDS_LEFT_SIDE;
                }
            else if (grabData(
                    GameDataType.SWITCH) == Position.RIGHT)
                {
                visionAuto = centerState.TURN_TOWARDS_RIGHT_SIDE;
                }
            else
                {
                visionAuto = centerState.DONE;
                }
            break;
        case TURN_TOWARDS_LEFT_SIDE:
            // Turn 90 degrees to the left, if the switch is on the left
            // sets state to BRAKE_2_L
            if (Hardware.autoDrive.turnDegrees(-90,
                    AUTO_SPEED_VISION) == true)
                {
                visionAuto = centerState.BRAKE_2_L;
                }
            break;
        case TURN_TOWARDS_RIGHT_SIDE:
            // Turn 90 degrees to the right, if the switch is on the right
            // sets state to BRAKE_2_L
            if (Hardware.autoDrive.turnDegrees(90,
                    AUTO_SPEED_VISION) == true)
                {
                visionAuto = centerState.BRAKE_2_R;
                }
        case BRAKE_2_L:
            // brake switch is on left
            // sets state to DRIVE_STRAIGHT_TO_SWITCH_LEFT
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_LEFT;
                }
            break;
        case BRAKE_2_R:
            // brake switch is on right
            // sets state to DRIVE_STRAIGHT_TO_SWITCH_RIGHT
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                {
                visionAuto = centerState.DRIVE_STRAIGHT_TO_SWITCH_RIGHT;
                }
            break;
        case DRIVE_STRAIGHT_TO_SWITCH_LEFT:
            // drive straight, switch is on the left then brakes
            // sets state to TURN_AGAIN_LEFT
            if (Hardware.autoDrive.driveStraightInches(
                    DRIVE_NO_CAMERA_LEFT - Hardware.autoDrive
                            .getBrakeStoppingDistance(),
                    AUTO_SPEED_VISION) == true)
                {
                if (Hardware.autoDrive
                        .brake(BrakeType.AFTER_DRIVE) == true)
                    visionAuto = centerState.TURN_AGAIN_LEFT;
                }
            break;
        case DRIVE_STRAIGHT_TO_SWITCH_RIGHT:
            // drive straight, switch is on the right then brakes
            // sets state to TURN_AGAIN_RIGHT
            if (Hardware.autoDrive.driveStraightInches(
                    DRIVE_NO_CAMERA_RIGHT - Hardware.autoDrive
                            .getBrakeStoppingDistance(),
                    AUTO_SPEED_VISION) == true)
                {
                if (Hardware.autoDrive
                        .brake(BrakeType.AFTER_DRIVE) == true)
                    visionAuto = centerState.TURN_AGAIN_RIGHT;
                }
            break;
        case TURN_AGAIN_RIGHT:
            // turn 90 to the right and sets the state to DRIVE_WITH_CAMERA
            if (Hardware.autoDrive.turnDegrees(90,
                    AUTO_SPEED_VISION) == true)
                {
                if (Hardware.autoDrive
                        .brake(BrakeType.AFTER_TURN) == true)
                    visionAuto = centerState.DRIVE_WITH_CAMERA;
                }
            break;
        case TURN_AGAIN_LEFT:
            // turns 90 to the left then brakes and sets the state to
            // DRIVE_WITH_CAMERA
            if (Hardware.autoDrive.turnDegrees(90,
                    AUTO_SPEED_VISION) == true)
                {
                if (Hardware.autoDrive
                        .brake(BrakeType.AFTER_TURN) == true)
                    visionAuto = centerState.LIFT; // TODO change back to DRIVE_WITH_CAMERA
                }
            break;
        case DRIVE_WITH_CAMERA:
            // drives to the switch based on the camera
            // sets state to LIFT
            if (Hardware.driveWithCamera.driveToSwitch(
                    AUTO_SPEED_VISION) == true)
                visionAuto = centerState.LIFT;
            break;
        case LIFT:
            // moves the forklift to the scale height and holds it there
            // sets state to DEPLOY_ARM
            if (Hardware.cubeManipulator.setLiftPosition(
                    SCALE_LIFT_HEIGHT, FORKLIFT_SPEED) == true)
                visionAuto = centerState.DEPLOY_ARM;
            break;
        case DEPLOY_ARM:
            // TODO this might be able to go before LIFT, or at the same time
            // as LIFT; also deployCubeIntake() can be called once early to get
            // it to start deploying, then it can be called again here to check
            // if it is finished
            // deploys cube intake and then sets state to MAKE_DEPOSIT
            Hardware.transmission.stop();
            if (Hardware.cubeManipulator.deployCubeIntake() == true)
                visionAuto = centerState.MAKE_DEPOSIT;
            break;
        case MAKE_DEPOSIT:
            // deposits cube on switch and sets state to DONE
            Hardware.transmission.stop();
            if (Hardware.cubeManipulator.scoreSwitch() == true)
                visionAuto = centerState.DONE;
            break;
        default: // prints that we reached the default case, then falls through
                 // to DONE
            Hardware.transmission.stop();
            System.out.println(
                    "REACHED THE DEFAULT CASE FOR centerSwitchPath()");
        case DONE:
            // stops robot the robot, and return true so the main auto
            // switch machine knows this path is done
            Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE);
            Hardware.autoDrive.driveInches(0, 0);
            return true;
        }
    return false;
}

public static centerState visionAuto = centerState.CENTER_INIT;

/**
 * Possible states for center vision autonomous
 * 
 * @author Becky Button
 *
 */
public static enum centerState
    {

CENTER_INIT, DRIVE_TEN_INCHES, BRAKE_1, GRAB_DATA,
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
 * Left side auto, turn 90 degrees to the right
 */
TURN_AGAIN_LEFT,
/**
 * Right side auto, turn 90 degrees to the left
 */
TURN_AGAIN_RIGHT,
/**
 * Drives with camera, then stops see driveToSwitch() in drive.
 */
DRIVE_WITH_CAMERA,
/**
 * Raises the lift SWITCH_LIFT_HEIGHT
 */
LIFT,
/**
 * Deploy the intake mechanism
 */
DEPLOY_ARM,
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
            currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE1;
            Hardware.cubeManipulator.deployCubeIntake();
            break;
        case DRIVE1:
            // FIRST driveInches: drive forward to switch

            // if we've finished driving this segment
            if (Hardware.autoDrive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[0]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                {
                // If the switch IS on the correct side, brake before turning
                // towards it
                if (robotPosition == Position.RIGHT && grabData(
                        GameDataType.SWITCH) == Position.RIGHT)
                    // if we're on the right side and the switch is too
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE1;
                else if (robotPosition == Position.LEFT && grabData(
                        GameDataType.SWITCH) == Position.LEFT)
                    // if we're on the left side and the switch is too
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE1;
                else
                    // If not, then keep driving forwards.
                    currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_BRAKING_DISTANCE_B4_DRIVE2;

                } // end if
            break;
        case BRAKE_DRIVE1:
            // Brake before turning towards the switch
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN1;
            break;
        case TURN1:
            // Turn towards the switch
            if (robotPosition == Position.RIGHT)
                {
                // We are on the Right side? turn left.
                if (Hardware.autoDrive.turnDegrees(-90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_TURN1;
                } // end if
            else
                {
                // We are on the Left side? turn right.
                if (Hardware.autoDrive.turnDegrees(90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_TURN1;
                } // end if
            break;
        case BRAKE_TURN1:
            // Brake after turning, and before going to the switch with
            // ultrasonic
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.RAISE_ARM1;
            break;
        case RAISE_ARM1: // TODO this case was changed and IS UNTESTED!!!
            // TODO do we want to start to raising the forklift early so it's
            // already up?
            // Raise the forklift/ arm for the switch after turning towards it,
            // but before
            // driving to the wall.
            if (Hardware.cubeManipulator
                    .setLiftPosition(SWITCH_LIFT_HEIGHT) == true)
                {
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_WITH_ULTRSNC;
                }
            break;
        case DRIVE_WITH_ULTRSNC:
            Hardware.autoDrive.driveStraight(DRIVE_SPEED, true);
            // Drive to the switch until the ultrasonic tells us to stop
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() < MIN_ULTRSNC_DISTANCE)
                currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_ULTRSNC;

            break;
        case BRAKE_ULTRSNC:
            // Brake after driving using the ultrasonic
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                {
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

        case DRIVE_BRAKING_DISTANCE_B4_DRIVE2:
            if (Hardware.autoDrive.driveStraightInches(
                    Hardware.autoDrive.getBrakeStoppingDistance(),
                    DRIVE_SPEED))
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE2;
            break;

        case DRIVE2:
            // Drive past the switch to the middle of the platform zone
            if (Hardware.autoDrive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[1]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE2;
            break;

        case BRAKE_DRIVE2:
            // Brake after we get to the middle of the platform zone
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN2;
            break;

        case TURN2:
            // Turn towards the platform zone
            if (robotPosition == Position.RIGHT)
                {
                // We are on the Right side? turn left.
                if (Hardware.autoDrive.turnDegrees(-90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_TURN2;
                }
            else
                {
                // We are on the Left side? turn right.
                if (Hardware.autoDrive.turnDegrees(90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_TURN2;
                }
            break;

        case BRAKE_TURN2:
            // Brake after turning towards the platform zone
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE3;
            break;

        case DRIVE3:
            // Drive to the right scale position
            if (Hardware.autoDrive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[2]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                // We start on the right side and the scale is on the right side
                if (robotPosition == Position.RIGHT && grabData(
                        GameDataType.SCALE) == Position.RIGHT)
                    {
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE3;
                    }
                // We start on the left side and the scale is on the left side
                else if (robotPosition == Position.LEFT && grabData(
                        GameDataType.SCALE) == Position.LEFT)
                    {
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE3;
                    }
                // No? adventure on...
                else
                    {
                    currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE_BRAKING_DISTANCE_B4_DRIVE4;
                    }
            break;
        case BRAKE_DRIVE3:
            // Brake after driving to the right side scale
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN3;
            break;
        case TURN3:
            // Turn towards the right switch
            if (robotPosition == Position.RIGHT)
                {
                // We start on the RIGHT side? turn right.
                if (Hardware.autoDrive.turnDegrees(90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_B4_RAISE_ARM2;
                }
            else
                {
                // We start on the LEFT side? turn left.
                if (Hardware.autoDrive.turnDegrees(-90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_B4_RAISE_ARM2;
                }
            break;

        case DRIVE_BRAKING_DISTANCE_B4_DRIVE4:
            if (Hardware.autoDrive.driveStraightInches(
                    Hardware.autoDrive.getBrakeStoppingDistance(),
                    DRIVE_SPEED))
                currentSwitchOrScaleState = SwitchOrScaleStates.DRIVE4;
            break;

        case DRIVE4:
            // Drive across the platform zone to the left
            if (Hardware.autoDrive.driveStraightInches(
                    SWITCH_OR_SCALE_DRIVE_DISTANCE[3]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_DRIVE4;
            break;
        case BRAKE_DRIVE4:
            // Brake after driving through the platform zone
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.TURN4;
            break;
        case TURN4:
            // Turn towards the scale
            if (robotPosition == Position.RIGHT)
                {
                // We start on the RIGHT side? turn right.
                if (Hardware.autoDrive.turnDegrees(90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_B4_RAISE_ARM2;
                }
            else
                {
                // We start on the LEFT side? turn left.
                if (Hardware.autoDrive.turnDegrees(-90,
                        TURN_SPEED) == true)
                    currentSwitchOrScaleState = SwitchOrScaleStates.BRAKE_B4_RAISE_ARM2;
                }
            break;

        case BRAKE_B4_RAISE_ARM2:
            // Brake right before we finish the auto path
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.RAISE_ARM2;
            break;
        case RAISE_ARM2:
            // Raise the arm to the scale height to set up for teleop.
            if (Hardware.cubeManipulator.setLiftPosition(
                    SCALE_LIFT_HEIGHT, FORKLIFT_SPEED) == true)
                currentSwitchOrScaleState = SwitchOrScaleStates.FINISH;
            break;

        default: // prints that we reached the default, then falls through to
                 // FINISH
            System.out.println(
                    "REACHED THE DEFAULT CASE for switchOrScalePath()");
        case FINISH:
            // if we don't know what's going on (default case) or we're
            // finished, stop driving and return true so the main autonomous
            // state can continue
            Hardware.transmission.stop();
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
PATH_INIT, DRIVE1, BRAKE_DRIVE1, TURN1, BRAKE_TURN1, RAISE_ARM1, DRIVE_WITH_ULTRSNC, BRAKE_ULTRSNC, EJECT_CUBE, DRIVE2, BRAKE_DRIVE2, TURN2, BRAKE_TURN2, DRIVE3, BRAKE_DRIVE3, TURN3, DRIVE4, BRAKE_DRIVE4, TURN4, BRAKE_B4_RAISE_ARM2, RAISE_ARM2, DRIVE_BRAKING_DISTANCE_B4_DRIVE2, DRIVE_BRAKING_DISTANCE_B4_DRIVE4, FINISH
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
            Hardware.autoDrive.setDefaultAcceleration(
                    DRIVE_STRAIGHT_ACCELERATION_TIME);
            currentOffsetSwitchState = OffsetSwitchPath.DRIVE1;
            // tell the cube intake mechanism to deploy
            Hardware.cubeManipulator.deployCubeIntake();
            break;

        case DRIVE1:
            // Drive forward a little to allow turning.
            if (Hardware.autoDrive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[0]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE1;
            break;
        case BRAKE_DRIVE1:
            // Brake after driving forwards
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentOffsetSwitchState = OffsetSwitchPath.TURN1;

            break;
        case TURN1:
            // Turn either left or right based on the game data provided
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                // If the switch is on the left side, then turn left
                if (Hardware.autoDrive.turnDegrees(-90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN1;
                }
            else
                {
                // If the switch is on the right side, then turn right
                if (Hardware.autoDrive.turnDegrees(90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN1;
                }
            break;
        case BRAKE_TURN1:
            // Brake after the first turn
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
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
            if (Hardware.autoDrive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[1]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE2;
            break;
        case DRIVE2R:
            // Drive if during turn 1, we turned right.
            if (Hardware.autoDrive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[2]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE2;
            break;
        case BRAKE_DRIVE2:
            // Brake after the 2nd drive function
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentOffsetSwitchState = OffsetSwitchPath.TURN2;
            break;
        case TURN2:
            // Turn after driving parallel to the driver station wall
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                // switch is on the left side? turn right
                if (Hardware.autoDrive.turnDegrees(90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN2;
                }
            else
                {
                // Switch is on the right side? turn left
                if (Hardware.autoDrive.turnDegrees(-90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN2;
                }
            break;
        case BRAKE_TURN2:
            // Brake after turning towards the opposing alliance
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                currentOffsetSwitchState = OffsetSwitchPath.RAISE_ARM;
            break;
        case RAISE_ARM:

            // tell the forklift to start raising up so we can drop off
            // the cube on the switch later
            Hardware.cubeManipulator.setLiftPosition(SWITCH_LIFT_HEIGHT,
                    FORKLIFT_SPEED);
            currentOffsetSwitchState = OffsetSwitchPath.DRIVE3;
            break;
        case DRIVE3:
            // Drive to the middle of the end of the switch
            if (Hardware.autoDrive.driveStraightInches(
                    OFFSET_SWITCH_DRIVE_DISTANCES[3]
                            - Hardware.autoDrive
                                    .getBrakeStoppingDistance(),
                    DRIVE_SPEED) == true)
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_DRIVE3;
            break;
        case BRAKE_DRIVE3:
            // Brake after driving to the center of the switch
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                currentOffsetSwitchState = OffsetSwitchPath.TURN3;
            break;
        case TURN3:
            // Turn towards the switch
            if (grabData(GameDataType.SWITCH) == Position.LEFT)
                {
                // Switch is on the left side? turn right.
                if (Hardware.autoDrive.turnDegrees(90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN3;
                }
            else
                {
                // Switch is on the right side? turn left.
                if (Hardware.autoDrive.turnDegrees(-90,
                        TURN_SPEED) == true)
                    currentOffsetSwitchState = OffsetSwitchPath.BRAKE_TURN3;
                }
            break;
        case BRAKE_TURN3:
            // Brake after turning towards the switch
            if (Hardware.autoDrive.brake(BrakeType.AFTER_TURN) == true)
                currentOffsetSwitchState = OffsetSwitchPath.DRIVE_WITH_ULTRSNC;
            break;
        case DRIVE_WITH_ULTRSNC:
            // Drive towards the switch using the ultrasonic
            Hardware.autoDrive.driveStraight(.3, true);
            if (Hardware.frontUltraSonic
                    .getDistanceFromNearestBumper() < MIN_ULTRSNC_DISTANCE)
                {
                Hardware.transmission.stop();
                currentOffsetSwitchState = OffsetSwitchPath.BRAKE_B4_EJECT;
                }
            break;
        case BRAKE_B4_EJECT:
            // Brake after driving with the ultrasonic, before we eject the cube
            if (Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                Hardware.autoTimer.reset();
                Hardware.autoTimer.start();
                currentOffsetSwitchState = OffsetSwitchPath.EJECT;
                }
            break;
        case EJECT:
            // Eject the cube we currently have into the switch :)
            Hardware.cubeIntakeMotor.set(-INTAKE_SPEED);
            if (Hardware.autoTimer.get() > INTAKE_EJECT_TIME)
                {
                Hardware.autoTimer.stop();
                Hardware.cubeIntakeMotor.stopMotor();
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
PATH_INIT, DEPLOY_INTAKE, DRIVE1, BRAKE_DRIVE1, TURN1, BRAKE_TURN1, DRIVE2L, DRIVE2R, BRAKE_DRIVE2, TURN2, BRAKE_TURN2, DRIVE3, RAISE_ARM_AND_DRIVE3_2, BRAKE_DRIVE3, TURN3, BRAKE_TURN3, RAISE_ARM, DRIVE_WITH_ULTRSNC, BRAKE_B4_EJECT, EJECT, FINISH
    }


/*
 * ================================
 * Constants
 * ================================
 */

// DRIVING
private static final double AUTO_TESTING_SCALAR = .5; // percent

private static final double DRIVE_STRAIGHT_ACCELERATION_TIME = .6; // seconds

private static final double DRIVE_SPEED = .5; // percent

private static final double TURN_SPEED = .4; // percent

// ==========

// OPERATING
private static final double FORKLIFT_SPEED = .5;

private static final double INTAKE_SPEED = 1;

private static final double INTAKE_EJECT_TIME = 1;// Seconds
// ==========

// LIFT HEIGHT
private static final int SWITCH_LIFT_HEIGHT = 24;// Inches

private static final int SCALE_LIFT_HEIGHT = 78;// Inches
// ==========

// ULTRASONIC
private static final int MIN_ULTRSNC_DISTANCE = 15;// Inches
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
private final static int DISTANCE_BACK_ACROSS_AUTOLINE = 100;

private final static int LEFT_DISTANCE_TO_EXCHANGE = 58;

private final static int LEFT_SIDE_TURN_TOWARDS_EXCHANGE = 90;

private final static int RIGHT_SIDE_TURN_TOWARDS_EXCHANGE = -90;

private final static int RIGHT_DISTANCE_TO_EXCHANGE = 130;

// CENTER_SWITCH
private final static int DRIVE_NO_CAMERA_LEFT = 53;

private final static int DRIVE_NO_CAMERA_RIGHT = 50;

private final static double CENTER_ACCEL = .6;

// TODO change for actual auto speed
private final static double AUTO_SPEED_VISION = .5;

// SWITCH_OR_SCALE
// array for storing the different driving distances in SWITH_OR_SCALE
private static final int[] SWITCH_OR_SCALE_DRIVE_DISTANCE = new int[]
    {(int) (AUTO_TESTING_SCALAR * 133),
            (int) (AUTO_TESTING_SCALAR * 67),
            (int) (AUTO_TESTING_SCALAR * 31),
            (int) (AUTO_TESTING_SCALAR * 169)};




// OFFSET_SWITCH
// array for storing the different driving distances used in OFFSET_SWITCH
// values in the array (in order) are for DRIVE1, DRIVE2L, DRIVE2R, DRIVE3
private static final int[] OFFSET_SWITCH_DRIVE_DISTANCES = new int[]
    {(int) (AUTO_TESTING_SCALAR * 6),
            (int) (AUTO_TESTING_SCALAR * 180),
            (int) (AUTO_TESTING_SCALAR * 59),
            (int) (AUTO_TESTING_SCALAR * 127)};


// FINISH



} // end class

