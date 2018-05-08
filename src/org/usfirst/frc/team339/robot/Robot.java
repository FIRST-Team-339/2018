/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// =======================================================a=============
// FILE NAME: Kilroy.java (Team 339 - Kilroy)
//
// CREATED ON: Oct 19, 2012
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// autonomousInit() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// disabledInit() - Initialization code for disabled mode should
// go here. This function will be called one time when the
// robot first enters disabled mode.
// robotInit() - Robot-wide initialization code should go here.
// It will be called exactly 1 time.
// teleopInit() - Initialization code for teleop mode should go here.
// Will be called each time the robot enters teleop mode.
// -----------------------------------------------------
// autonomousPeriodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// disabledPeriodic() - Periodic code for disabled mode should go here.
// Will be called periodically at a regular rate while the robot
// is in disabled mode.
// teleopPeriodic() - Periodic code for teleop mode should go here.
// Will be called periodically at a regular rate while the robot
// is in teleop mode.
// -----------------------------------------------------
// autonomousContinuous() - Continuous code for autonomous mode should
// go here. Will be called repeatedly as frequently as possible
// while the robot is in autonomous mode.
// disabledContinuous() - Continuous code for disabled mode should go
// here. Will be called repeatedly as frequently as possible while
// the robot is in disabled mode.
// teleopContinuous() - Continuous code for teleop mode should go here.
// Will be called repeatedly as frequently as possible while the
// robot is in teleop mode.
// -----------------------------------------------------
// Other functions not normally used
// startCompetition() - This function is a replacement for the WPI
// supplied 'main loop'. This should not normally be written or
// used.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.Utils.drive.Drive.BrakeType;

import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
/**
 * -------------------------------------------------------
 * declares all the code necessary to extend the IterativeRobot class. These are
 * all the methods needed to run Kilroy during a match
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
public class Robot extends IterativeRobot
{
// =================================================
// private data for the class
// =================================================
/**
 * -------------------------------------------------------
 * Initialization code for autonomous mode should go here. Will be called
 * once when the robot enters autonomous mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 * 
 *          -------------------------------------------------------
 */
@Override
public void autonomousInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started AutonousInit().");

    // =========================================================
    // User code goes below here
    // =========================================================
    if (Hardware.demoModeSwitch.isOn() == false)
        Autonomous.init();

    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed AutonousInit().");
} // end autonomousInit

/**
 * -------------------------------------------------------
 * Non-User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 * This in turn calls the Autonomous class's Periodic function, which is
 * where the user code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 * 
 *          -------------------------------------------------------
 */
@Override
public void autonomousPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================
    if (Hardware.demoModeSwitch.isOn() == false)
        Autonomous.periodic();
    // =========================================================
    // User code goes above here
    // =========================================================
}// end autonomousPeriodic

/**
 * -------------------------------------------------------
 * Initialization code for disabled mode should go here. Will be called once
 * when the robot enters disabled mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void disabledInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started DisabledInit().");
    // =========================================================
    // User code goes below here
    // =========================================================
    Hardware.tempRelay.set(false);
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed DisabledInit().");
} // end disabledInit

/**
 * -------------------------------------------------------
 * Periodic code for disabled mode should go here. Will be called
 * periodically at a regular rate while the robot is in disabled mode. Code
 * that can be "triggered" by a joystick button can go here. This can set up
 * configuration things at the driver's station for instance before a match.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void disabledPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================
} // end disabledPeriodic

/**
 * -------------------------------------------------------
 * This function is run when the robot is first started up and should be
 * used for any initialization code for the robot.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void robotInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started robotInit()");

    // =========================================================
    // User code goes below here
    // =========================================================

    // REMOVE ME!!!
    // Hardware.autoDrive.setGyro(Hardware.gyroAnalog);

    // -------------------------------------
    // Resets encoder values
    // -------------------------------------
    Hardware.rightFrontDriveEncoder.setInverted(false);
    Hardware.leftFrontDriveEncoder.setInverted(false);
    Hardware.rightRearDriveEncoder.setInverted(false);
    Hardware.leftRearDriveEncoder.setInverted(false);
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.intakeDeployEncoder.reset();
    Hardware.liftingEncoder.reset();
    Hardware.liftingEncoder.setReverseDirection(true);
    Hardware.intakeDeployEncoder.reset();

    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------
    Hardware.leftDriveMotor.setSafetyEnabled(false);
    Hardware.rightDriveMotor.setSafetyEnabled(false);
    Hardware.liftingMotor.setSafetyEnabled(false);
    Hardware.cubeIntakeMotor.setSafetyEnabled(false);
    Hardware.intakeDeployArm.setSafetyEnabled(false);

    Hardware.intakeDeployArm.setInverted(true);
    Hardware.cubeIntakeMotor.setInverted(true);

    // -------------------------------------
    // Manually sets encoders Distance per Pulse
    // -------------------------------------
    if (Hardware.on2018 == true)
        {
        Hardware.autoDrive.setEncoderDistancePerPulse(
                KILROY_XIX_ENCODER_DPP,
                MotorPosition.ALL);

        Hardware.liftingEncoder
                .setDistancePerPulse(KILROY_XIX_LIFT_ENCODER_DPP);
        Hardware.intakeDeployEncoder
                .setDistancePerPulse(KILROY_XIX_DEPLOY_ENCODER_DPP);

        Hardware.transmission.setAllGearRatios(KILROY_XIX_GEAR_1_SPEED,
                KILROY_XIX_GEAR_2_SPEED);

        Hardware.frontUltraSonic.setOffsetDistanceFromNearestBumper(
                KILROY_XIX_US_DISTANCE_FROM_BUMPER);
        Hardware.rearUltraSonic.setOffsetDistanceFromNearestBumper(
                KILROY_XIX_US_DISTANCE_FROM_BUMPER);
        // Hardware.ringLightRelay.setDirection(Direction.kForward);

        // Drive Functions

        Hardware.autoDrive.setTurningRadius(KILROY_XIX_TURNING_RADIUS);

        // Braking constants
        Hardware.autoDrive.setBrakePower(KILROY_XIX_BRAKE_DRIVE_POWER,
                BrakeType.AFTER_DRIVE);
        Hardware.autoDrive.setBrakePower(KILROY_XIX_BRAKE_TURN_POWER,
                BrakeType.AFTER_TURN);

        Hardware.autoDrive.setBrakeDeadband(
                KILROY_XIX_BRAKE_DRIVE_DEADBAND, BrakeType.AFTER_DRIVE);
        Hardware.autoDrive.setBrakeDeadband(
                KILROY_XIX_BRAKE_TURN_DEADBAND, BrakeType.AFTER_TURN);
        Hardware.autoDrive.setDriveStraightConstant(
                KILROY_XIX_DRIVESTRAIGHT_CONSTANT);

        Hardware.USBCam.setResolution(320, 240);
        Hardware.USBCam.setFPS(20);
        Hardware.USBCam.setPixelFormat(VideoMode.PixelFormat.kYUYV);
        // Hardware.USBCamUp.setResolution(320, 240);
        // Hardware.USBCamUp.setFPS(20);
        // Hardware.USBCamUp.setPixelFormat(VideoMode.PixelFormat.kYUYV);
        } // end if
    else
        {
        Hardware.autoDrive.setEncoderDistancePerPulse(
                KILROY_XV_ENCODER_DPP,
                MotorPosition.ALL);

        Hardware.liftingEncoder
                .setDistancePerPulse(KILROY_XIX_LIFT_ENCODER_DPP);
        Hardware.intakeDeployEncoder
                .setDistancePerPulse(KILROY_XIX_DEPLOY_ENCODER_DPP);

        Hardware.transmission.setAllGearRatios(KILROY_XV_GEAR_1_SPEED,
                KILROY_XV_GEAR_2_SPEED);
        Hardware.autoDrive.setTurningRadius(KILROY_XV_TURNING_RADIUS);
        // Hardware.ringLightRelay.setDirection(Direction.kReverse);
        } // else

    // ----------------------------------
    // For last years robot - do the following
    // ----------------------------------
    if (Hardware.onNessie == true)
        {
        Hardware.leftDriveMotor.setInverted(false);
        Hardware.rightDriveMotor.setInverted(false);
        Hardware.transmission.setAllGearRatios(NESSIE_GEAR_1_SPEED,
                NESSIE_GEAR_2_SPEED);
        } // if
    else
    // ----------------------------------
    // this years configuration
    // ----------------------------------
        {
        Hardware.leftDriveMotor.setInverted(false);
        Hardware.rightDriveMotor.setInverted(true);
        } // else

    // ---------------------------------
    // Sets joystick deadband
    // ---------------------------------
    Hardware.transmission.setJoystickDeadband(JOYSTICK_DEADBAND_RANGE);

    // ---------------------------------
    // Sets the angle of the servo to 100
    // ---------------------------------
    Hardware.climbingMechanismServo.set(CLIMB_SERVO_INIT_POSITION);

    // ---------------------------------
    // sets the ring light to off
    // ---------------------------------
    // Hardware.ringLightRelay.set(Value.kOff);

    if (Hardware.demoModeSwitch.isOn())
        {
        Hardware.cubeManipulator
                .setMaxLiftHeight(demoForkliftMaxHeight);
        }

    Hardware.gyro.calibrate();
    Hardware.gyro.reset();
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    Teleop.printStatements();

    System.out.println(
            "Kilroy XIX is started.  All hardware items created.");
} // end robotInit()

/**
 * ------------------------------------------------------
 * Non-User initialization code for teleop mode should go here. Will be
 * called once when the robot enters teleop mode, and will call the Teleop
 * class's Init function, where the User code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void teleopInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started teleopInit().");
    // =========================================================
    // User code goes below here
    // =========================================================
    Teleop.init();

    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed TeleopInit().");
} // end teleopInit

/**
 * -------------------------------------------------------
 * Non-User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode, and
 * will in turn call the Teleop class's Periodic function.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void teleopPeriodic ()
{
    // -------------------------------------
    // Call the Teleop class's Periodic function,
    // which contains the user code.
    // -------------------------------------

    // =========================================================
    // User code goes below here
    // =========================================================
    Teleop.periodic();

    // =========================================================
    // User code goes above here
    // =========================================================

} // end teleopPeriodic

/**
 * -------------------------------------------------------
 * Initialization code for test mode should go here. Will be called once
 * when the robot enters test mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015
 *          ------------------------------------------------------
 */
@Override
public void testInit ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================

} // end testInit

/**
 * -------------------------------------------------------
 * Periodic code for test mode should go here. Will be called periodically
 * at a regular rate while the robot is in test mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015
 *          -------------------------------------------------------
 */
@Override
public void testPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================

} // end testPeriodic

// ==========================================
// TUNEABLES
// ==========================================
private static final int demoForkliftMaxHeight = 49;

public static final double demoForkliftSpeed = .5;

private static final double KILROY_XV_ENCODER_DPP = .0174;

private static final double KILROY_XIX_ENCODER_DPP = 0.0346;

private static final double KILROY_XIX_GEAR_1_SPEED = .4;

private static final double KILROY_XV_GEAR_1_SPEED = .3;

private static final double NESSIE_GEAR_1_SPEED = .5;

public static final double KILROY_XIX_GEAR_2_SPEED = .6;

public static final double KILROY_XV_GEAR_2_SPEED = .6;

private static final double NESSIE_GEAR_2_SPEED = .5;

private static final double JOYSTICK_DEADBAND_RANGE = .2;

private static final double KILROY_XIX_LIFT_ENCODER_DPP = 0.02;

private static final double KILROY_XIX_DEPLOY_ENCODER_DPP = .1;

private static final double KILROY_XIX_TURNING_RADIUS = 11.5 - .35;

private static final double KILROY_XV_TURNING_RADIUS = 11 - .35;

// Brake stuff
private static final double KILROY_XIX_BRAKE_DRIVE_POWER = .09;

private static final double KILROY_XIX_BRAKE_TURN_POWER = .2;

private static final int KILROY_XIX_BRAKE_TURN_DEADBAND = 15;

private static final int KILROY_XIX_BRAKE_DRIVE_DEADBAND = 20;

// Drive Straight
private static final double KILROY_XIX_DRIVESTRAIGHT_CONSTANT = .1;

private static final int KILROY_XIX_US_DISTANCE_FROM_BUMPER = 3;


// Servo Constants
// position the climb servo starts in
public static final double CLIMB_SERVO_INIT_POSITION = 1.0;

// position the climb servo goes to when the root is climbing
public static final double CLIMB_SERVO_CLIMB_POISITION = 0.0;

} // end class

