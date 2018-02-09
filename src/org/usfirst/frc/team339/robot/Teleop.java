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
import org.usfirst.frc.team339.HardwareInterfaces.transmission.Drive.BrakeType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    // User code goes below here

    // ---------------------------------
    // Encoder resetting
    // ---------------------------------
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.intakeDeployEncoder.reset();

    // ---------------------------------
    // setup motors
    // ---------------------------------
    Hardware.transmission.setForTeleop(Robot.GEAR_2_SPEED);
    Hardware.rightDriveMotor.set(0);
    Hardware.leftDriveMotor.set(0);
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

    if (Hardware.leftOperator.getRawButton(6) == true)
        Hardware.cubeManipulator.deployCubeIntake();

    if (Hardware.leftOperator.getRawButton(7) == true)
        Hardware.cubeManipulator.retractCubeIntake();

    if (Hardware.leftOperator.getRawButton(8))
        {
        isTestinfForklift = true;
        forkliftState = 0;
        }

    if (Hardware.leftOperator.getRawButton(7))
        {
        isTestinfForklift = true;
        forkliftState = 0;
        }

    // NOT WORKING IN cubeManipulator
    // scoreScale
    // scoreSwitch


    if (isTestinfForklift == true)
        {
        System.out.println("lifting motor position: "
                + Hardware.liftingMotor.getPosition());
        System.out.println("lifting motor speed: "
                + Hardware.liftingMotor.getSpeed());
        System.out.println("Forklift height: "
                + Hardware.cubeManipulator.getForkliftHeight());
        if (Hardware.cubeManipulator.scoreScale()
                && forkliftState == 0)
            {

            }


        }



    if (Hardware.leftOperator.getRawButton(9) == true)
        {
        allowAlignment = true;
        }
    if (Hardware.leftOperator.getRawButton(10))
        {
        allowAlignment = false;
        }

    if (allowAlignment == true)
        {
        // Hardware.cubeManipulator.forkliftUpdate();
        // Hardware.cubeManipulator.setLiftPosition(80, .9);
        Hardware.transmission.setForAutonomous();
        if (Hardware.scaleAlignment.alignToScale(.3, 3))
            {
            System.out.println("aligned to scale");
            Hardware.transmission
                    .setForTeleop(Robot.GEAR_2_SPEED);
            allowAlignment = false;
            }
        }
    // -----------------------------------------
    // Forklift controls
    // -----------------------------------------
    if (allowAlignment == false)
        {
        Hardware.cubeManipulator.masterUpdate();

        Hardware.cubeManipulator
                .moveForkliftWithController(Hardware.rightOperator);

        // Intake controls
        Hardware.cubeManipulator
                .intakeCube(Hardware.rightOperator.getRawButton(2));

        Hardware.cubeManipulator
                .intakeCubeOverride(
                        Hardware.rightOperator.getRawButton(4));

        // Push out the cube
        Hardware.cubeManipulator
                .pushOutCubeTeleop(
                        Hardware.rightOperator.getRawButton(3));

        // Set intake/deploy motor to position based on encoder w/ Momentary
        // Switch
        if (Hardware.deployIntakeButton.isOnCheckNow() == true)
            Hardware.cubeManipulator.deployCubeIntake();
        }
    // Set Servo to position w/ Momentary Switch
    if (Hardware.climbButton.isOnCheckNow() == true)
        Hardware.climbingMechanismServo.setAngle(CLIMBING_SERVO_ANGLE);

    // =================================================================
    // CAMERA CODE
    // =================================================================

    // =================================================================
    // DRIVING CODE
    // =================================================================
    // if the right driver button 3 is pressed, shift up a gear, if the left
    // driver button 3 id pressed, shift down a gear
    Hardware.transmission.shiftGears(
            Hardware.rightDriver.getRawButton(3),
            Hardware.leftDriver.getRawButton(3));

    // if is testing drive is equal to true, the joysticks are locked out to
    // test some sort of drive function (of drive by camera)

    if (isTestingDrive == false && allowAlignment == false)
        Hardware.transmission.drive(Hardware.leftDriver,
                Hardware.rightDriver);


    // ------------------------------------
    // print out any information needed to
    // display on the drivers station
    // ------------------------------------
    printStatements();

    // -------------------------------------------
    // Put anything you need to test, but the
    // code will not be a part of the final teleop
    // -------------------------------------------
    // testingDrive();

    beckyTest();

} // end Periodic()

private static boolean allowAlignment = false;

private static boolean isTestingDrive = false;

private static boolean isTestinfForklift = false;

private static int forkliftState = 0;

private static int driveState = 0;

public static void beckyTest ()
{
    if (Hardware.onNessie == true)
        {
        // if (Hardware.visionTestButton.isOnCheckNow())
        // {
        // Hardware.transmission.setForAutonomous();
        // if (Hardware.driveWithCamera.driveToSwitch(.5) == true)
        // {
        //
        // Hardware.autoDrive.driveInches(0, 0);
        // }
        // System.out.println("The center is: " +
        // Hardware.driveWithCamera.getCameraCenterValue());
        // }
        // Hardware.ringLightRelay.set(Value.kForward);
        // Hardware.axisCamera.saveImage(ImageType.RAW);

        // Hardware.driveWithCamera.getCameraCenterValue();
        if (Hardware.onNessie == true)
            {
            isTestingDrive = true;
            }
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
        }
} // end beckyTest()

private static void testingDrive ()
{
    if (Hardware.leftDriver.getRawButton(9) == true)
        {
        isTestingDrive = true;
        }

    if (isTestingDrive)
        {
        Hardware.transmission.setForAutonomous();
        Hardware.autoDrive.setDefaultAcceleration(.5);
        if (driveState == 0
                && Hardware.autoDrive.driveStraightInches(60, -.5))
            {
            driveState++;
            }
        else if (driveState == 1
                && Hardware.autoDrive.brake(BrakeType.AFTER_DRIVE))
            {
            driveState++;
            }

        if (Hardware.leftDriver.getRawButton(10) || driveState == 2)
            {
            Hardware.transmission.stop();
            System.out.println("LDistance: "
                    + Hardware.leftFrontDriveEncoder.getDistance());
            System.out.println("RDistance: "
                    + Hardware.rightFrontDriveEncoder.getDistance());
            driveState = 0;
            Hardware.transmission.setForTeleop(Robot.GEAR_2_SPEED);
            isTestingDrive = false;
            }
        }

    if (Hardware.leftDriver.getRawButton(8) == true)
        Hardware.autoDrive.resetEncoders();

} // end of testingDrive()

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
    // System.out.println(
    // "Right Drive Motor " + Hardware.rightDriveMotor.get());
    // SmartDashboard.putNumber("R Drive Motor",
    // Hardware.rightDriveMotor.get());
    // System.out.println(
    // "Left Drive Motor " + Hardware.leftDriveMotor.get());
    // SmartDashboard.putNumber("L Drive Motor",
    // Hardware.leftDriveMotor.get());
    // System.out.println("Lifting Motor " + Hardware.liftingMotor.get());
    // SmartDashboard.putNumber("Lifting Motor",
    // Hardware.liftingMotor.get());
    // System.out.println(
    // "Cube Intake Motor " + Hardware.cubeIntakeMotor.get());
    // SmartDashboard.putNumber("Cube Motor",
    // Hardware.cubeIntakeMotor.get());
    // System.out.println(
    // "Intake Deploy Arm " + Hardware.intakeDeployArm.get());
    // SmartDashboard.putNumber("Intake Deploy Motor",
    // Hardware.intakeDeployArm.get());
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
    // SmartDashboard.putBoolean("Disable SW",
    // Hardware.disableAutonomousSwitch.isOn());

    // if (Hardware.leftAutoSwitch.isOn() == false)
    // System.out.println(
    // "Left = off");
    // else
    // System.out.println(
    // "Left = on");
    // SmartDashboard.putBoolean("L Auto SW",
    // Hardware.leftAutoSwitch.isOn());
    //
    // if (Hardware.rightAutoSwitch.isOn() == false)
    // System.out.println(
    // "Right = off");
    // else
    // System.out.println(
    // "Right = on");
    // SmartDashboard.putBoolean("R Auto SW",
    // Hardware.rightAutoSwitch.isOn());
    //
    // System.out.println("6 pos = "
    // + Hardware.autoSixPosSwitch.getPosition());
    // SmartDashboard.putNumber("6 Pos Switch",
    // Hardware.leftFrontDriveEncoder.get());
    //
    // ---------------------------------
    // Encoders
    // ---------------------------------
    // System.out.println("Left Front Encoder Inches = "
    // + Hardware.leftFrontDriveEncoder.getDistance());
    // SmartDashboard.putNumber("Left Front Encoder Inches",
    // Hardware.leftFrontDriveEncoder.getDistance());

    // System.out.println("Left Front Encoder Ticks "
    // + Hardware.leftFrontDriveEncoder.get());
    // SmartDashboard.putNumber("Left Front Encoder Ticks",
    // Hardware.leftFrontDriveEncoder.get());

    // System.out.println("Right Front Inches = "
    // + Hardware.rightFrontDriveEncoder.getDistance());
    // SmartDashboard.putNumber("Right Front Encoder Inches",
    // Hardware.rightFrontDriveEncoder.getDistance());

    // System.out.println("Right Front Ticks "
    // + Hardware.rightFrontDriveEncoder.get());
    // SmartDashboard.putNumber("Right Front Encoder Ticks",
    // Hardware.rightFrontDriveEncoder.get());

    // System.out.println("Left Rear Encoder Inches = "
    // + Hardware.leftRearDriveEncoder.getDistance());
    // SmartDashboard.putNumber("Left Rear Encoder Inches",
    // Hardware.leftRearDriveEncoder.getDistance());

    // System.out.println("Left Rear Encoder Ticks "
    // + Hardware.leftRearDriveEncoder.get());
    // SmartDashboard.putNumber("Left Rear Encoder Ticks",
    // Hardware.leftRearDriveEncoder.get());

    // System.out.println("Right Rear Inches = "
    // + Hardware.rightRearDriveEncoder.getDistance());
    // SmartDashboard.putNumber("Right Rear Encoder Inches",
    // Hardware.rightRearDriveEncoder.getDistance());

    // System.out.println("Right Rear Ticks "
    // + Hardware.rightRearDriveEncoder.get());
    // SmartDashboard.putNumber("Right Rear Encoder Ticks",
    // Hardware.rightRearDriveEncoder.get());

    // System.out.println(
    // "Lift Encoder Inches = "
    // + Hardware.liftingEncoder.getDistance());
    // SmartDashboard.putNumber("Lift Encoder Inches",
    // Hardware.liftingEncoder.getDistance());

    // System.out.println(
    // "Lift Encoder Ticks " + Hardware.liftingEncoder.get());
    // SmartDashboard.putNumber("Lift Encoder Ticks",
    // Hardware.liftingEncoder.getDistance());

    // System.out.println("Intake Deploy Encoder "
    // + Hardware.intakeDeployEncoder.getDistance());
    // SmartDashboard.putNumber("Intake Deploy Encoder",
    // Hardware.intakeDeployEncoder.getDistance());

    // System.out.println("Intake Deploy Encoder Ticks "
    // + Hardware.intakeDeployEncoder.get());
    // SmartDashboard.putNumber("Intake Deploy Ticks",
    // Hardware.intakeDeployEncoder.get());

    // ---------------------------------
    // Red Light/IR Sensors
    // prints the state of the sensor
    // ---------------------------------
    // System.out
    // .println("Right Red Light " + Hardware.rightRedLight.isOn());
    // SmartDashboard.putBoolean("R Red Light",
    // Hardware.rightRedLight.isOn());
    // System.out.println("Left Red Light " + Hardware.leftRedLight.isOn());
    // SmartDashboard.putBoolean("L Red Light",
    // Hardware.leftRedLight.isOn());
    // System.out.println(
    // "PhotoSwitch " + Hardware.cubePhotoSwitch.isOn());
    // SmartDashboard.putBoolean("Photo SW",
    // Hardware.cubePhotoSwitch.isOn());

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
    // SmartDashboard.putNumber("Delay Pot",
    // Hardware.delayPot.get(0, 5));

    // --------------------------
    // Sonar/UltraSonic
    // --------------------------
    // System.out.println("Front UltraSonic "
    // + Hardware.frontUltraSonic.getDistanceFromNearestBumper());
    // SmartDashboard.putNumber("Front Ultrasonic",
    // Hardware.frontUltraSonic.getDistanceFromNearestBumper());
    // System.out.println("Rear UltraSonic "
    // + Hardware.rearUltraSonic.getDistanceFromNearestBumper());
    // SmartDashboard.putNumber("Read Ultrasonic",
    // Hardware.rearUltraSonic.getDistanceFromNearestBumper());

    // =========================
    // Servos
    // =========================
    // System.out.println("Climbing Mechanism Servo" +
    // Hardware.climbingMechanismServo.getAngle());
    // SmartDashboard.putNumber("Climb Servo",
    // Hardware.climbingMechanismServo.getAngle());

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
    // System.out.println("The camera center is: " +
    // Hardware.autoDrive.getCameraCenterValue());
    // SmartDashboard.putNumber("Camera Center",
    // Hardware.driveWithCamera.getCameraCenterValue());

    // =================================
    // Driver station
    // =================================

    // ---------------------------------
    // Joysticks
    // information about the joysticks
    // ---------------------------------
    // System.out.println(
    // "Right Driver Joystick " + Hardware.rightDriver.getY());
    // SmartDashboard.putNumber("R Driver Y Joy",
    // Hardware.rightDriver.getY());
    // System.out.println(
    // "Left Driver Joystick " + Hardware.leftDriver.getY());
    // SmartDashboard.putNumber("L Driver Y Joy",
    // Hardware.leftDriver.getY());
    // System.out.println(
    // "Right Operator Joystick " + Hardware.rightOperator.getY());
    // SmartDashboard.putNumber("R Operator Y Joy",
    // Hardware.rightOperator.getY());
    // System.out.println(
    // "Left Operator Joystick " + Hardware.leftOperator.getY());
    // SmartDashboard.putNumber("L Operator Y Joy",
    // Hardware.leftOperator.getY());

    // =================================
    // KILROY ANCILLARY ITEMS
    // =================================
    // ---------------------------------
    // Gear number displayed to driver
    // ---------------------------------
    // System.out.println(
    // "Gear = " + Hardware.transmission.getCurrentGear() + 1);
    // SmartDashboard.putNumber("Gear",
    // Hardware.transmission.getCurrentGear() + 1);

    // ---------------------------------
    // timers
    // what time does the timer have now
    // ---------------------------------
    SmartDashboard.updateValues();
} // end printStatements()

// ================================
// Constants
// ================================
public static final int CLIMBING_SERVO_ANGLE = 78;

} // end class
