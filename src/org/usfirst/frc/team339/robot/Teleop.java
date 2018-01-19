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
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for teleop mode
// should go here. Will be called each time the robot enters
// teleop mode.
// -----------------------------------------------------
// Periodic() - Periodic code for teleop mode should
// go here. Will be called periodically at a regular rate while
// the robot is in teleop mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import org.usfirst.frc.team339.Hardware.Hardware;

/**
 * This class contains all of the user code for the Autonomous part of the
 * match, namely, the Init and Periodic code
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Teleop
{
/**
 * User Initialization code for teleop mode should go here. Will be called
 * once when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */

public static void init ()
{


} // end Init

// tune pid loop
/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void periodic ()
{

    // =================================================================
    // OPERATOR CONTROLS
    // =================================================================

    // =================================================================
    // CAMERA CODE
    // =================================================================

    // =================================================================
    // Driving code
    // =================================================================

    if (isTestingDrive == false)
        Hardware.tractionDrive.drive(Hardware.leftDriver.getY(),
                Hardware.rightDriver.getY());

    if (Hardware.leftDriver.getRawButton(9))
        isTestingDrive = true;

    if (isTestingDrive == true)
        {

        if (testingDriveState == 0)
            if (Hardware.autoDrive.driveInches(36, .6))
                testingDriveState++;
        if (testingDriveState == 1)
            if (Hardware.autoDrive.brake())
                {
                testingDriveState = 0;
                isTestingDrive = false;
                }

        if (Hardware.leftDriver.getRawButton(10))
            {
            testingDriveState = 0;
            isTestingDrive = false;
            }

        }


    printStatements();
}
// end
// Periodic


private static boolean isTestingDrive = false;

private static int testingDriveState = 0;

/**
 * stores print statements for future use in the print "bank", statements
 * are commented out when not in use, when you write a new print statement,
 * "deposit" the statement in the correct "bank" do not "withdraw"
 * statements, unless directed to.
 * 
 * NOTE: Keep the groupings below, which correspond in number and order as
 * the hardware declarations in the HARDWARE class
 * 
 * @author Ashley Espeland
 * @written 1/28/16
 * 
 *          Edited by Ryan McGee Also Edited by Josef Liebl
 * 
 */
public static void printStatements ()
{

    // =================================
    // Motor
    // Prints the value of motors
    // =================================

    //
    // System.out.println(
    // "Right Drive Motor " + Hardware.rightDriveMotor.get());
    // System.out.println(
    // "Left Drive Motor " + Hardware.leftDriveMotor.get());
    // System.out.println("Lifting Motor " + Hardware.liftingMotor.get());
    // System.out.println(
    // "Cube Intake Motor " + Hardware.cubeIntakeMotor.get());
    // System.out.println(
    // "Intake Deploy Arm " + Hardware.intakeDeployArm.get());
    //
    // =================================
    // CAN items
    // prints value of the CAN controllers
    // =================================
    //
    // =================================
    // Relay
    // =================================
    // System.out.println(
    // "Ring Light Relay " + Hardware.ringLightRelay.get());
    //
    // =================================
    // // Digital Inputs
    // =================================
    //
    // ---------------------------------
    // Switches
    // prints state of switches
    // ---------------------------------
    // if (Hardware.disableAutonomousSwitch.isOn() == false)
    // System.out.println(
    // "Disable Auto Switch is off");
    // else
    // System.out.println(
    // "Disable Auto Switch is on");
    //
    // if (Hardware.leftAutoSwitch.isOn() == false)
    // System.out.println(
    // "Left Auto Switch is off");
    // else
    // System.out.println(
    // "Left Auto Switch is on");
    //
    // if (Hardware.rightAutoSwitch.isOn() == false)
    // System.out.println(
    // "Right Auto Switch is off");
    // else
    // System.out.println(
    // "Right Auto Switch is on");
    //
    //
    // ---------------------------------
    // Encoders

    // System.out.println("Left Front Drive Encoder Distance "
    // + Hardware.leftFrontDriveEncoder.getDistance());

    System.out.println("Left Front Encoder Ticks "
            + Hardware.leftFrontDriveEncoder.get());

    // System.out.println("Right Front Drive Encoder Distance "
    // + Hardware.rightFrontDriveEncoder.getDistance());

    System.out.println("Right Front Drive Encoder Ticks "
            + Hardware.rightFrontDriveEncoder.get());

    // System.out.println("Left Rear Drive Encoder Distance "
    // + Hardware.leftRearDriveEncoder.getDistance());

    System.out.println("Left Rear Drive Encoder Ticks "
            + Hardware.leftRearDriveEncoder.get());

    // System.out.println("Right Rear Drive Encoder Distance "
    // + Hardware.rightRearDriveEncoder.getDistance());

    System.out.println("Right Rear Drive Encoder Ticks"
            + Hardware.rightRearDriveEncoder.get());


    // System.out.println(
    // "Lifting Encoder Distance "
    // + Hardware.liftingEncoder.getDistance());

    // System.out.println(
    // "Lifting Encoder Ticks" + Hardware.liftingEncoder.get());

    // System.out.println("Intake Deploy Encoder "
    // + Hardware.intakeDeployEncoder.getDistance());

    // System.out.println("Intake Deploy Encoder Ticks "
    // + Hardware.intakeDeployEncoder.get());

    // ---------------------------------

    // ---------------------------------
    // Red Light/IR Sensors
    // prints the state of the sensor
    // ---------------------------------

    //
    // System.out
    // .println("Right Red Light " + Hardware.rightRedLight.get());
    // System.out.println("Left Red Light " + Hardware.leftRedLight.get());
    // System.out.println(
    // "Cube Photo Switch " + Hardware.cubePhotoSwitch.get());
    //
    // =================================
    // Pneumatics
    // =================================

    // ---------------------------------
    // Compressor
    // prints information on the compressor
    // ---------------------------------

    // ---------------------------------
    // Solenoids
    // prints the state of solenoids
    // ---------------------------------

    // =================================
    // Analogs
    // =================================

    // ---------------------------------
    // pots
    // where the pot is turned to
    // ---------------------------------

    // System.out
    // .println("Delay Potentiometer " + Hardware.delayPot.get());

    // --------------------------
    // Sonar/UltraSonic
    // --------------------------
    // System.out.println("Front UltraSonic "
    // + Hardware.frontUltraSonic.getDistanceFromNearestBumper());
    // System.out.println("Rear UltraSonic "
    // + Hardware.rearUltraSonic.getDistanceFromNearestBumper());

    // =========================
    // Servos
    // =========================
    // System.out.println("Climbing Mechanism Servo" +
    // Hardware.climbingMechanismServo.get());
    //
    // ================
    // GYRO
    // =================

    // =================================
    // Connection Items
    // =================================

    // ---------------------------------
    // Cameras
    // prints any camera information required
    // ---------------------------------

    // =================================
    // Driver station
    // =================================

    // ---------------------------------
    // Joysticks
    // information about the joysticks
    // ---------------------------------
    // System.out.println(
    // "Right Driver Joystick " + Hardware.rightDriver.getY());
    // System.out.println(
    // "Left Driver Joystick " + Hardware.leftDriver.getY());
    // System.out.println(
    // "Right Operator Joystick " + Hardware.rightOperator.getY());
    // System.out.println(
    // "Left Operator Joystick " + Hardware.leftOperator.getY());
    // =================================
    // Kilroy ancillary items
    // =================================

    // ---------------------------------
    // timers
    // what time does the timer have now
    // ---------------------------------

} // end printStatements

/*
 * ================================
 * Constants
 * ================================
 */

} // end class
