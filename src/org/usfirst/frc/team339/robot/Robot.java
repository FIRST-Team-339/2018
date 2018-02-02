/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
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
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
// -------------------------------------------------------
/**
 * declares all the code necessary to extend the IterativeRobot class. These are
 * all the methods needed to run Kilroy during a match
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
public class Robot extends IterativeRobot
{
// =================================================
// private data for the class
// =================================================
// -------------------------------------------------------

/**
 * Initialization code for autonomous mode should go here. Will be called
 * once when the robot enters autonomous mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
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
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.intakeDeployEncoder.reset();
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

// -------------------------------------------------------
/**
 * Non-User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 * This in turn calls the Autonomous class's Periodic function, which is
 * where the user code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void autonomousPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================
    Autonomous.periodic();
    // =========================================================
    // User code goes above here
    // =========================================================

}// end autonomousPeriodic
 // -------------------------------------------------------

/**
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

    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed DisabledInit().");
} // end disabledInit

// -------------------------------------------------------
/**
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
    // -------------------------------------
    // Watch dog code used to go here.
    // -------------------------------------
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================
} // end disabledPeriodic
  // -------------------------------------------------------

/**
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

    // Resets encoder values
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.intakeDeployEncoder.reset();
    Hardware.liftingEncoder.reset();
    Hardware.liftingEncoder.setReverseDirection(false);
    Hardware.intakeDeployEncoder.reset();

    if (Hardware.onNessie == false)
        {
        // Nessie Settings
        Hardware.leftDriveMotor.setInverted(false);
        Hardware.rightDriveMotor.setInverted(false);

        }
    else
        {
        // Ball Bot Settings
        Hardware.leftDriveMotor.setInverted(false);
        Hardware.rightDriveMotor.setInverted(true);
        }
    // Sets max gears on robot
    Hardware.tractionDrive.setMaxGears(2);

    // This sets the gear speed percentage for the traction drive
    Hardware.tractionDrive.setAllGearRatios(GEAR_1_SPEED, GEAR_2_SPEED);
    Hardware.tractionDrive.setJoystickDeadband(JOYSTICK_DEADBAND_RANGE);

    // Sets all encoders Distance per pulse
    Hardware.autoDrive.setEncoderDistancePerPulse(
            KILROY_XIX_ENCODER_DPP,
            MotorPosition.ALL);

    // Manually sets encoders Distance per Pulse
    Hardware.leftFrontDriveEncoder
            .setDistancePerPulse(KILROY_XIX_ENCODER_DPP);
    Hardware.rightFrontDriveEncoder
            .setDistancePerPulse(KILROY_XIX_ENCODER_DPP);
    Hardware.leftRearDriveEncoder
            .setDistancePerPulse(KILROY_XIX_ENCODER_DPP);
    Hardware.rightRearDriveEncoder
            .setDistancePerPulse(KILROY_XIX_ENCODER_DPP);

    Hardware.liftingEncoder
            .setDistancePerPulse(KILROY_XIX_LIFT_ENCODER_DPP);
    Hardware.intakeDeployEncoder
            .setDistancePerPulse(KILROY_XIX_DEPLOY_ENCODER_DPP);

    // Sets the angle of the servo to 100
    Hardware.climbingMechanismServo.setAngle(200);

    // sets the ring light to off
    Hardware.ringLightRelay.set(Value.kReverse);



    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println(
            "Kilroy XIX is started.  All hardware items created.");
} // end robotInit

// -------------------------------------------------------
/**
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
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.intakeDeployEncoder.reset();
    Teleop.init();

    Hardware.rightDriveMotor.set(0);
    Hardware.leftDriveMotor.set(0);



    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed TeleopInit().");
} // end teleopInit

// -------------------------------------------------------
/**
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

    Teleop.periodic();

    // User code goes above here
    // =========================================================

} // end teleopPeriodic
  // -------------------------------------------------------

/**
 * Initialization code for test mode should go here. Will be called once
 * when the robot enters test mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015
 *          -------------------------------------------------------
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
  // -------------------------------------------------------

/**
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

private static final double KILROY_XIX_ENCODER_DPP = .0174;

private static final double GEAR_1_SPEED = .5;

private static final double GEAR_2_SPEED = .7;

private static final double JOYSTICK_DEADBAND_RANGE = .2;

private static final double KILROY_XIX_LIFT_ENCODER_DPP = .1;

private static final double KILROY_XIX_DEPLOY_ENCODER_DPP = .1;


} // end class

