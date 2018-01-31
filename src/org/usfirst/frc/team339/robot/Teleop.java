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
import edu.wpi.first.wpilibj.Timer;

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
    totalLoopTime = 0.0;
    numOfLoops = 1;
    teleopLoopTimer.reset();
    teleopLoopTimer.start();


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


    Hardware.cubeManipulator
            .pushOutCubeTeleop(Hardware.rightOperator.getRawButton(3));



    //
    // Forklift controls
    // Hardware.cubeManipulator
    // .moveForkliftWithController(Hardware.rightOperator);


    // if (Hardware.leftOperator.getRawButton(2) == true
    // && Hardware.cubeManipulator.moveLiftDistance(50,
    // .3) == false)
    // {
    // Hardware.cubeManipulator.moveLiftDistance(50);
    // System.out.println("WE DID THE THING");
    // }
    // else
    // {
    // Hardware.cubeManipulator.stopForklift();
    // }

    Hardware.cubeManipulator
            .moveForkliftWithController(Hardware.rightOperator);

    // // intake controls
    Hardware.cubeManipulator
            .intakeCube(Hardware.rightOperator.getRawButton(2));


    //

    Hardware.cubeManipulator
            .intakeCubeOverride(Hardware.rightOperator.getRawButton(4));

    if (Hardware.climbButton.isOnCheckNow() == true)
        {
        Hardware.climbingMechanismServo.setAngle(CLIMBING_SERVO_ANGLE);
        }

    //

    //
    // =================================================================
    // CAMERA CODE
    // =================================================================

    // test from 1/23/18
    // if (Hardware.visionTestButton.isOnCheckNow())
    // {
    // Hardware.autoDrive.driveToSwitch(1.3, .6);
    // }
    //

    // Hardware.ringLightRelay.set(Value.kForward);
    // if (Hardware.visionTestButton.isOnCheckNow())
    // {
    // Hardware.axisCamera.processImage();
    // Hardware.autoDrive.visionTest(1.3, .6);
    // Hardware.axisCamera.saveImage(ImageType.PROCESSED);
    // for (int i = 0; i < Hardware.axisCamera
    // .getParticleReports().length; i++)
    // {
    // System.out.println("The center of " + i + " is: "
    // + Hardware.axisCamera.getNthSizeBlob(i).center.x);
    // }
    // System.out.println("The center is : " + (Hardware.axisCamera
    // .getNthSizeBlob(0).center.x
    // + Hardware.axisCamera.getNthSizeBlob(1).center.x) / 2);
    // }


    // =================================================================
    // Driving code
    // =================================================================

    if (isTestingDrive == false
            && Hardware.leftDriver.getTrigger() == false)
        Hardware.tractionDrive.drive(Hardware.leftDriver,
                Hardware.rightDriver);


    Hardware.tractionDrive.shiftGears(
            Hardware.rightDriver.getRawButton(3),
            Hardware.leftDriver.getRawButton(3));

    if (Hardware.leftDriver.getRawButton(9))
        isTestingDrive = true;

    if (isTestingDrive)
        {
        if (driveState == 0
                && Hardware.autoDrive.driveStraightInches(36, .5))
            driveState++;
        else if (driveState == 1 && Hardware.autoDrive.brake())
            driveState++;

        if (Hardware.leftDriver.getRawButton(10) || driveState == 2)
            {
            Hardware.tractionDrive.stop();
            driveState = 0;
            Hardware.autoDrive.resetEncoders();
            isTestingDrive = false;
            }

        }


    // NOTE - CLAIRE TEST NEXT MEETING
    if (Hardware.rightOperator.getRawButton(2)) // 2 is a placeholder
        {
        Hardware.climbingMechanismServo.setAngle(110);
        }

    printStatements();

    // totalLoopTime += teleopLoopTimer.get();
    // teleopLoopTimer.reset();
    // averageLoopTime = totalLoopTime / numOfLoops;
    // numOfLoops++;

} // end Periodic

private static boolean isTestingDrive = false;

private static int driveState = 0;

// timer to keep track of how long it spent to get through this loop of teleop
private static Timer teleopLoopTimer = new Timer();

// average time it has taken to loop through teleop (or more accurately, robot
// as a whole)
private static double averageLoopTime = 0.0;

// total time since beginning of teleop init
private static double totalLoopTime = 0.0;

// number of times we'll started through teleop periodic (starts at 1)
private static int numOfLoops = 1;

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
    // int Gear = Hardware.tractionDrive.getCurrentGear() + 1;


    // Gear shift status
    // System.out.println(
    // "Gear = " + Gear);


    // =================================
    // Motor
    // Prints the value of motors
    // =================================

    // System.out.println(
    // "Right Drive Motor " + Hardware.rightDriveMotor.get());
    // System.out.println(
    // "Left Drive Motor " + Hardware.leftDriveMotor.get());
    // System.out.println("Lifting Motor " + Hardware.liftingMotor.get());
    // System.out.println(
    // "Cube Intake Motor " + Hardware.cubeIntakeMotor.get());
    // System.out.println(
    // "Intake Deploy Arm " + Hardware.intakeDeployArm.get());


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


    // System.out.println("Left Front Encoder Inches = "
    // + Hardware.leftFrontDriveEncoder.getDistance());

    // System.out.println("Left Front Encoder Ticks "
    // + Hardware.leftFrontDriveEncoder.get());

    // System.out.println("Right Front Inches = "
    // + Hardware.rightFrontDriveEncoder.getDistance());

    // System.out.println("Right Front Ticks "
    // + Hardware.rightFrontDriveEncoder.get());

    // System.out.println("Left Rear Encoder Inches = "
    // + Hardware.leftRearDriveEncoder.getDistance());

    // System.out.println("Left Rear Encoder Ticks "
    // + Hardware.leftRearDriveEncoder.get());

    // System.out.println("Right Rear Inches = "
    // + Hardware.rightRearDriveEncoder.getDistance());

    // System.out.println("Right Rear Ticks "
    // + Hardware.rightRearDriveEncoder.get());

    // System.out.println(
    // "Lift Encoder Inches = "
    // + Hardware.liftingEncoder.getDistance());

    // System.out.println(
    // "Lift Encoder Ticks " + Hardware.liftingEncoder.get());

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
    System.out.println(
            "PhotoSwitch " + Hardware.cubePhotoSwitch.isOn());
    //
    // =================================
    // Pneumatics----------------------------------
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
    // System.out.println("Front UltraSonic "
    // + Hardware.frontUltraSonic.getDistanceFromNearestBumper());
    // System.out.println("Rear UltraSonic "
    // + Hardware.rearUltraSonic.getDistanceFromNearestBumper());
    //
    // =========================
    // Servos
    // =========================
    // System.out.println("Climbing Mechanism Servo" +
    // Hardware.climbingMechanismServo.getAngle());
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

    // System.out.println(
    // "\n" + "LOOP TIMER: " + teleopLoopTimer.get() + "; "
    // + "avg: " +
    // averageLoopTime + "\n");

} // end printStatements

//
// ================================
// Constants
// ================================
//

public static final int CLIMBING_SERVO_ANGLE = 78;

} // end class
