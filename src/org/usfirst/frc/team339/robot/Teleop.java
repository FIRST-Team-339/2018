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

    Hardware.cubeManipulator.forkliftUpdate();

    // Forklift controls
    Hardware.cubeManipulator
            .moveForkliftWithController(Hardware.rightOperator);
    // intake controls
    if (Hardware.rightOperator.getRawButton(2) == true)
        {
        Hardware.cubeManipulator.intakeCube();
        }

    if (Hardware.rightOperator.getRawButton(3) == true)
        {
        Hardware.cubeManipulator.pushOutCube();
        }
    // =================================================================
    // CAMERA CODE
    // =================================================================

    // test from 1/23/18
    // if (Hardware.visionTestButton.isOnCheckNow())
    // {
    // Hardware.ringLightRelay.set(Value.kOn);
    // Hardware.autoDrive.driveToSwitch(1.5, .3);
    // }

    // Hardware.ringLightRelay.set(Value.kForward);

    // =================================================================
    // Driving code
    // =================================================================

    if (isTestingDrive == false)
        Hardware.tractionDrive.drive(Hardware.leftDriver.getY(),
                Hardware.rightDriver.getY());
    printStatements();
} // end Periodic


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
    // "R Drive Motor " + Hardware.rightDriveMotor.get());
    // System.out.println(
    // "L Drive Motor " + Hardware.leftDriveMotor.get());
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
    // "Relay " + Hardware.ringLightRelay.get());
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
    // "Disable = off");
    // else
    // System.out.println(
    // "Disable = on");
    //
    // if (Hardware.leftAutoSwitch.isOn() == false)
    // System.out.println(
    // "Left = off");
    // else
    // System.out.println(
    // "Left = on");
    //
    // if (Hardware.rightAutoSwitch.isOn() == false)
    // System.out.println(
    // "Right = off");
    // else
    // System.out.println(
    // "Right = on");
    //
    //
    // System.out.println("6 pos = "
    // + Hardware.autoSixPosSwitch.getPosition());
    //
    // ---------------------------------
    // Encoders

    // System.out.println("LF In = "
    // + Hardware.leftFrontDriveEncoder.getDistance());

    // System.out.println("LF Ticks "
    // + Hardware.leftFrontDriveEncoder.get());

    // System.out.println("RF In = "
    // + Hardware.rightFrontDriveEncoder.getDistance());

    // System.out.println("RF Ticks "
    // + Hardware.rightFrontDriveEncoder.get());

    // System.out.println("LR In = "
    // + Hardware.leftRearDriveEncoder.getDistance());

    // System.out.println("LR Ticks "
    // + Hardware.leftRearDriveEncoder.get());

    // System.out.println("RR In = "
    // + Hardware.rightRearDriveEncoder.getDistance());

    // System.out.println("RR Ticks "
    // + Hardware.rightRearDriveEncoder.get());


    // System.out.println(
    // "Lift In = "
    // + Hardware.liftingEncoder.getDistance());

    // System.out.println(
    // "Lift Ticks " + Hardware.liftingEncoder.get());

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
    // .println("R Light " + Hardware.rightRedLight.get());
    // System.out.println("L Light " + Hardware.leftRedLight.get());
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
    // .println("Delay Pot " + Hardware.delayPot.get(0, 5));

    // --------------------------
    // Sonar/UltraSonic
    // --------------------------
    // System.out.println("F USonic "
    // + Hardware.frontUltraSonic.getDistanceFromNearestBumper());
    // System.out.println("R USonic "
    // + Hardware.rearUltraSonic.getDistanceFromNearestBumper());
    //
    // // =========================
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
    // "RDriver Joystick " + Hardware.rightDriver.getY());
    // System.out.println(
    // "LDriver Joystick " + Hardware.leftDriver.getY());
    // System.out.println(
    // "ROp Joystick " + Hardware.rightOperator.getY());
    // System.out.println(
    // "LOp Joystick " + Hardware.leftOperator.getY());
    // =================================
    // Kilroy ancillary items
    // =================================

    // ---------------------------------
    // timers
    // what time does the timer have now
    // ---------------------------------

} // end printStatements

//
// ================================
// Constants
// ================================
//

} // end class
