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
import org.usfirst.frc.team339.vision.VisionProcessor.ImageType;
import edu.wpi.first.wpilibj.Relay.Value;
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
    Hardware.takePictureTimer.reset();

    Hardware.tractionDrive.setForTeleop(Robot.GEAR_2_SPEED);

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


    // --------- all this is temporary testing code

    // if (Hardware.leftOperator.getRawButton(5))
    // {
    // System.out.println("Move to 10.0");
    // Hardware.cubeManipulator.moveLiftDistance(10.0);
    // }
    //
    // if (Hardware.leftOperator.getRawButton(8))
    // {
    // System.out.println("Move to 20.0");
    // Hardware.cubeManipulator.moveLiftDistance(20.0);
    // }
    //
    // if (Hardware.leftOperator.getRawButton(9))
    // {
    // System.out.println("Move to 30.0");
    // Hardware.cubeManipulator.moveLiftDistance(30.0);
    // }

    // ----------- end temporary testing code


    // -----------------------------------
    // forklift functionality - update every loop
    // to make sure that everything is updated on
    // all parts of the fork lift mechanism
    // -----------------------------------
    Hardware.cubeManipulator.forkliftUpdate();
    Hardware.cubeManipulator
            .moveForkliftWithController(Hardware.rightOperator);
    Hardware.cubeManipulator
            .intakeCube(Hardware.rightOperator.getRawButton(2));
    Hardware.cubeManipulator
            .intakeCubeOverride(Hardware.rightOperator.getRawButton(4));
    Hardware.cubeManipulator
            .pushOutCubeTeleop(Hardware.rightOperator.getRawButton(3));



    // Set Servo to position w/ Momentary Switch
    // if (Hardware.climbButton.isOnCheckNow() == true)
    // {
    // Hardware.climbingMechanismServo.setAngle(CLIMBING_SERVO_ANGLE);
    // }



    // Set intake/deploy motor to position based on encoder w/ Momentary Switch
    if (Hardware.deployIntakeButton.isOnCheckNow() == true)
        Hardware.cubeManipulator.deployCubeIntake();


    // takes a picture with the axis camera when button 7 on the left Operator
    // is pressed



    if (Hardware.leftOperator.getRawButton(6)
            && Hardware.leftOperator.getRawButton(7))


        // ------------------------------------
        // takes a picture with the axis camera when buttons 6 + 7 on the left
        // Operator is pressed
        // ------------------------------------
        if (Hardware.leftOperator.getRawButton(6) == true
                && Hardware.leftOperator.getRawButton(7) == true

                && pictureTakenByButton == false
                && takePictureByButton == false)
            {
            takePictureByButton = true;
            Hardware.takePictureTimer.start();
            } // end if

    if (!(Hardware.leftOperator.getRawButton(6) == true
            && Hardware.leftOperator.getRawButton(7)) == true
            && pictureTakenByButton == true)
        {
        takePictureByButton = false;
        pictureTakenByButton = false;
        } // end if


    if (takePictureByButton == true)
        {
        if (Hardware.takePictureTimer.get() <= TAKE_PICTURE_DELAY
                / 2.0)
            Hardware.ringLightRelay.set(Value.kForward);

        if (Hardware.takePictureTimer.get() >= TAKE_PICTURE_DELAY)
            {

            Hardware.axisCamera.saveImageSafely(
                    true,
                    ImageType.RAW);

            pictureTakenByButton = true;
            takePictureByButton = false;

            Hardware.axisCamera.saveImageSafely(
                    false,
                    ImageType.RAW);


            Hardware.ringLightRelay.set(Value.kReverse);

            Hardware.takePictureTimer.stop();
            Hardware.takePictureTimer.reset();

            } // end if
        } // end if

    // =================================================================
    // CAMERA CODE
    // =================================================================

    // =================================================================
    // Driving code
    // =================================================================

    if (isTestingDrive == false)
        Hardware.tractionDrive.drive(Hardware.leftDriver,
                Hardware.rightDriver);

    Hardware.tractionDrive.shiftGears(
            Hardware.rightDriver.getRawButton(3),
            Hardware.leftDriver.getRawButton(3));




    if (Hardware.leftDriver.getRawButton(9))
        isTestingDrive = true;

    if (isTestingScale)
        {
        Hardware.tractionDrive.setForAutonomous();

        if (Hardware.autoDrive.alignToScale(.2, 3))
            {
            System.out.println("Has aligned to scale?????");
            }








        if (isTestingDrive == false)
            {
            Hardware.tractionDrive.drive(Hardware.leftDriver,
                    Hardware.rightDriver);
            }
        Hardware.tractionDrive.shiftGears(
                Hardware.rightDriver.getRawButton(3),
                Hardware.leftDriver.getRawButton(3));


        if (Hardware.leftDriver.getRawButton(9))
            isTestingDrive = true;

        if (isTestingDrive)
            {

            if (driveState == 0
                    && Hardware.autoDrive.driveInches(36, .5))
                driveState++;
            else if (driveState == 1 && Hardware.autoDrive.brake())
                driveState++;

            if (Hardware.leftDriver.getRawButton(10) || driveState == 2)
                {
                Hardware.tractionDrive.stop();
                driveState = 0;
                Hardware.tractionDrive.setForTeleop(Robot.GEAR_2_SPEED);
                isTestingDrive = false;
                }

            }



        // NOTE - CLAIRE TEST NEXT MEETING
        if (Hardware.rightOperator.getRawButton(2)) // 2 is a placeholder
            {
            Hardware.climbingMechanismServo.setAngle(110);
            }



        // NOTE - CLAIRE TEST NEXT MEETING
        if (Hardware.rightOperator.getRawButton(2)) // 2 is a placeholder
            Hardware.climbingMechanismServo.setAngle(110);
        // ---------------------------------------
        // Becky's vision testing code
        // ---------------------------------------
        if (Hardware.onNessie == true)

            beckyTest();
        // --------------------------------------
        // all print statements for all hardware items
        // --------------------------------------
        printStatements();
        }

} // end Periodic

private static void testingDrive ()
{

    // Button 9 starts the drive test
    if (Hardware.leftDriver.getRawButton(9))
        isTestingDrive = true;

    if (isTestingDrive)
        {
        // Setup the transmission for auto
        Hardware.tractionDrive.setForAutonomous();
        Hardware.autoDrive.setDefaultAcceleration(.5);
        // First state do this, second state brake.
        if (driveState == 0
                && Hardware.autoDrive.turnDegrees(90, .4))
            {
            Hardware.autoDrive.resetEncoders();
            driveState++;
            }
        else if (driveState == 1 && Hardware.autoDrive.brake())
            driveState++;

        // Button 10 is an override to stop moving
        if (Hardware.leftDriver.getRawButton(10) || driveState == 2)
            {
            Hardware.tractionDrive.stop();
            System.out.println("LDistance: "
                    + Hardware.leftFrontDriveEncoder.getDistance());
            System.out.println("RDistance: "
                    + Hardware.rightFrontDriveEncoder.getDistance());
            driveState = 0;
            Hardware.tractionDrive.setForTeleop(Robot.GEAR_2_SPEED);
            isTestingDrive = false;
            }
        }

    if (Hardware.leftDriver.getRawButton(8))
        Hardware.autoDrive.resetEncoders();

}

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

    System.out.println("flok lift heigth"
            + Hardware.cubeManipulator.getForkliftHeight());


    System.out.println(
            "intake motor speed" + Hardware.cubeIntakeMotor.getSpeed());


    // System.out.println("fork lift height: "
    // + Hardware.cubeManipulator.getForkliftHeight());
    //
    //
    // System.out.println(
    // "Intake motor speed" + Hardware.cubeIntakeMotor.getSpeed());




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
    // System.out.println(
    // "PhotoSwitch " + Hardware.cubePhotoSwitch.isOn());
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
    // System.out.println("The camera center is: " +
    // Hardware.autoDrive.getCameraCenterValue());
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


public static void beckyTest ()
{
    if (Hardware.visionTestButton.isOnCheckNow() == true)
        {
        Hardware.tractionDrive.setForAutonomous();
        if (Hardware.autoDrive.driveToSwitch(1.5, .5) == true)
            {
            Hardware.autoDrive.driveInches(0, 0);
            }
        }
    Hardware.ringLightRelay.set(Value.kForward);

    Hardware.axisCamera.saveImage(ImageType.PROCESSED);
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

//
// ================================
// Constants
// ================================
// --------------------------------
// public constants
// --------------------------------
// how many seconds we wait before taking a picture via buttons
public static final double TAKE_PICTURE_DELAY = 0.1;

public static final int CLIMBING_SERVO_ANGLE = 78;

// -------------------------------
// private constants
// -------------------------------
private static boolean isTestingDrive = false;

private static boolean isTestingScale = false;

private static boolean takePictureByButton = false;

private static boolean pictureTakenByButton = false;

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

} // end class Teleop
