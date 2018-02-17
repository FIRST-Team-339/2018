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
import org.usfirst.frc.team339.Utils.CubeManipulator;
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

    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------
    Hardware.leftDriveMotor.setSafetyEnabled(false);
    Hardware.rightDriveMotor.setSafetyEnabled(false);
    Hardware.liftingMotor.setSafetyEnabled(false);
    Hardware.cubeIntakeMotor.setSafetyEnabled(false);
    Hardware.intakeDeployArm.setSafetyEnabled(false);

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
    Hardware.transmission.setForTeleop(Robot.KILROY_XIX_GEAR_2_SPEED);
    Hardware.rightDriveMotor.set(0);
    Hardware.leftDriveMotor.set(0);

    SmartDashboard.putNumber("Deadband", 0);
    SmartDashboard.putNumber("Power", 0);

    // lift test init
    SmartDashboard.putNumber("Lifting motor speed", 0);

    SmartDashboard.putNumber("Cube intake speed", 0);

    SmartDashboard.putNumber("Intake deploy speed", 0);

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

    Hardware.cubeManipulator.intakeCube(
            Hardware.rightOperator.getTrigger(),
            Hardware.rightOperator.getRawButton(2),
            Hardware.rightOperator.getRawButton(3));

    // -----------------------------------------
    // Deploy Intake controls
    // -----------------------------------------
    // Button 11 to deploy, 10 to retract, and 9 for override for both.
    if (Hardware.leftOperator.getRawButton(11))
        Hardware.cubeManipulator.deployCubeIntake(
                Hardware.leftOperator.getRawButton(9));
    else if (Hardware.leftOperator.getRawButton(10))
        Hardware.cubeManipulator.retractCubeIntake(
                Hardware.leftOperator.getRawButton(9));
    // -----------------------------------------
    // Forklift (not Cube Manipulator) controls
    // -----------------------------------------

    Hardware.cubeManipulator
            .moveForkliftWithController(
                    Hardware.rightOperator.getY(),
                    Hardware.rightOperator.getRawButton(5));

    if (Hardware.rightOperator.getRawButton(6) == true)
        Hardware.cubeManipulator
                .setLiftPosition(CubeManipulator.SCALE_HEIGHT, .6);
    else if (Hardware.rightOperator.getRawButton(7) == true)
        Hardware.cubeManipulator
                .setLiftPosition(CubeManipulator.SWITCH_HEIGHT, .6);

    if (Hardware.climbButton.isOnCheckNow() == true)
        Hardware.climbingMechanismServo.setAngle(CLIMBING_SERVO_ANGLE);


    // update for the cube manipulator (forklift, intake, etc.) and its state
    // machines
    Hardware.cubeManipulator.masterUpdate();


    // =================================================================
    // CAMERA CODE
    // =================================================================

    Hardware.axisCamera
            .takeLitPicture(Hardware.leftOperator.getRawButton(6)
                    && Hardware.leftOperator.getRawButton(7));

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
    //
    // if (isTestingDrive == false && allowAlignment == false
    // && isTesting2StepTurn == false
    // && isTestingPivotTurn == false
    // && isTestingEncoderTurn == false && isBeckyTest == false
    // && Hardware.leftDriver.getTrigger() == false)

    Hardware.transmission.drive(Hardware.leftDriver,
            Hardware.rightDriver);
    // update

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

    // scaleTest();

    liftTest();
    // beckyTest();

} // end Periodic()

private static boolean inAligning = true;

private static boolean allowAlignment = false;

private static boolean isTestingDrive = false;

private static boolean isTestingAnalogGyroTurn = false;

private static boolean isTestingEncoderTurn = false;

private static boolean isTestingPivotTurn = false;

private static boolean isTesting2StepTurn = false;

private static int driveState = 0;


public static void scaleTest ()
{
    SmartDashboard.putNumber("RearUltraSonic",
            Hardware.rearUltraSonic.getDistanceFromNearestBumper());

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

        Hardware.transmission.setForAutonomous();
        if (Hardware.scaleAlignment.alignToScale(.3, 3))
            {
            System.out.println("aligned to scale");
            Hardware.transmission
                    .setForTeleop(Robot.KILROY_XIX_GEAR_2_SPEED);
            allowAlignment = false;
            inAligning = true;
            }
        }


}


private static boolean isBeckyTest = false;

private static void beckyTest ()
{
    // Hardware.ringLightRelay.set(Value.kForward);
    if (Hardware.rightOperator.getRawButton(2) == true)
        {
        Hardware.transmission.setForAutonomous();
        isBeckyTest = true;
        }

    if (isBeckyTest == true)
        {

        if (Hardware.driveWithCamera.driveToSwitch(.3) == true)
            {
            Hardware.transmission
                    .setForTeleop(Robot.KILROY_XV_GEAR_2_SPEED);
            isBeckyTest = false;
            }

        }

    // if (Hardware.rightOperator.getRawButton(2))
    // {
    // Hardware.tempRelay.set(true);
    // }
    // else
    // {
    // Hardware.tempRelay.set(false);
    // }
    //
    // if(Hardware.rightOperator.getRawButton(3))
    // {
    // Hardware.ringLightRelay.setDirection(Direction.kForward);
    // }
    // else
    // {
    // Hardware.ringLightRelay.setDirection(Direction.kReverse);
    // }



    // if (Hardware.leftOperator.getRawButton(7))
    // {
    // Hardware.ringLightRelay.set(Value.kForward);
    // }
    // else
    // {
    // Hardware.ringLightRelay.set(Value.kReverse);
    // }


    // Hardware.axisCamera.saveImage(ImageType.RAW);

    // Hardware.driveWithCamera.getCameraCenterValue();

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
} // end beckyTest()

private static void testingDrive ()
{
    Hardware.autoDrive.setBrakeDeadband(
            (int) SmartDashboard.getNumber("Deadband", 0),
            BrakeType.AFTER_DRIVE);
    Hardware.autoDrive
            .setBrakePower(SmartDashboard.getNumber("Power", 0),
                    BrakeType.AFTER_DRIVE);

    if (Hardware.leftDriver.getRawButton(9) == true)
        {
        isTestingDrive = true;
        }

    if (isTestingDrive == true)
        {
        Hardware.transmission.setForAutonomous();
        Hardware.autoDrive.setDefaultAcceleration(.5);
        if (isTestingDrive == true && driveState == 0
                && Hardware.autoDrive.driveStraightInches(48,
                        .3) == true)
            {
            driveState++;
            }
        else if (driveState == 1
                && Hardware.autoDrive
                        .brake(BrakeType.AFTER_DRIVE) == true)
            {
            driveState++;
            }

        if (Hardware.leftDriver.getRawButton(10) == true
                || driveState == 2)
            {
            Hardware.transmission.stop();
            driveState = 0;
            Hardware.transmission
                    .setForTeleop(Robot.KILROY_XV_GEAR_2_SPEED);
            Hardware.autoDrive.reset();
            isTestingDrive = false;
            isTestingEncoderTurn = false;
            isTestingPivotTurn = false;
            isTesting2StepTurn = false;
            isTestingAnalogGyroTurn = false;
            }
        }

    if (Hardware.leftDriver.getRawButton(8) == true)
        {
        Hardware.gyro.reset();
        }

} // end of testingDrive()

private static void liftTest ()
{
    // SmartDashboard.putNumber("throttle value : ",
    // Hardware.leftOperator.getThrottle());

    Hardware.liftingMotor
            .set(SmartDashboard.getNumber("Lifting motor speed", 0));

    Hardware.cubeIntakeMotor
            .set(SmartDashboard.getNumber("Cube intake speed", 0));

    Hardware.intakeDeployArm
            .set(SmartDashboard.getNumber("Intake deploy speed", 0));

    if (Hardware.leftOperator.getRawButton(3))
        {
        Hardware.tempRelay.set(true);
        }
    else
        {
        Hardware.tempRelay.set(false);
        }

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
    SmartDashboard.putNumber("Lifting Motor",
            Hardware.liftingMotor.get());

    // System.out.println(
    // "Cube Intake Motor " + Hardware.cubeIntakeMotor.get());
    SmartDashboard.putNumber("Cube Motor",
            Hardware.cubeIntakeMotor.get());
    // System.out.println(
    // "Intake Deploy Arm " + Hardware.intakeDeployArm.get());
    SmartDashboard.putNumber("Intake Deploy Motor",
            Hardware.intakeDeployArm.get());
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
    SmartDashboard.putBoolean("Disable SW",
            Hardware.disableAutonomousSwitch.isOn());

    // if (Hardware.leftAutoSwitch.isOn() == false)
    // System.out.println(
    // "Left = off");
    // else
    // System.out.println(
    // "Left = on");
    SmartDashboard.putBoolean("L Auto SW",
            Hardware.leftAutoSwitch.isOn());
    //
    // if (Hardware.rightAutoSwitch.isOn() == false)
    // System.out.println(
    // "Right = off");
    // else
    // System.out.println(
    // "Right = on");
    SmartDashboard.putBoolean("R Auto SW",
            Hardware.rightAutoSwitch.isOn());
    //
    // System.out.println("6 pos = "
    // + Hardware.autoSixPosSwitch.getPosition());
    SmartDashboard.putNumber("6 Pos Switch",
            Hardware.autoSixPosSwitch.getPosition());

    // ---------------------------------
    // Encoders
    // ---------------------------------
    // System.out.println("Left Front Encoder Inches = "
    // + Hardware.leftFrontDriveEncoder.getDistance());
    SmartDashboard.putNumber("Left Front Encoder Inches",
            Hardware.leftFrontDriveEncoder.getDistance());

    // System.out.println("Left Front Encoder Ticks "
    // + Hardware.leftFrontDriveEncoder.get());
    SmartDashboard.putNumber("Left Front Encoder Ticks",
            Hardware.leftFrontDriveEncoder.get());

    // System.out.println("Right Front Inches = "
    // + Hardware.rightFrontDriveEncoder.getDistance());
    SmartDashboard.putNumber("Right Front Encoder Inches",
            Hardware.rightFrontDriveEncoder.getDistance());

    // System.out.println("Right Front Ticks "
    // + Hardware.rightFrontDriveEncoder.get());
    SmartDashboard.putNumber("Right Front Encoder Ticks",
            Hardware.rightFrontDriveEncoder.get());

    // System.out.println("Left Rear Encoder Inches = "
    // + Hardware.leftRearDriveEncoder.getDistance());
    SmartDashboard.putNumber("Left Rear Encoder Inches",
            Hardware.leftRearDriveEncoder.getDistance());

    // System.out.println("Left Rear Encoder Ticks "
    // + Hardware.leftRearDriveEncoder.get());
    SmartDashboard.putNumber("Left Rear Encoder Ticks",
            Hardware.leftRearDriveEncoder.get());

    // System.out.println("Right Rear Inches = "
    // + Hardware.rightRearDriveEncoder.getDistance());
    SmartDashboard.putNumber("Right Rear Encoder Inches",
            Hardware.rightRearDriveEncoder.getDistance());

    // System.out.println("Right Rear Ticks "
    // + Hardware.rightRearDriveEncoder.get());
    SmartDashboard.putNumber("Right Rear Encoder Ticks",
            Hardware.rightRearDriveEncoder.get());

    // System.out.println("Lift Encoder Inches = "
    // + Hardware.liftingEncoder.getDistance());
    SmartDashboard.putNumber("Lift Encoder Inches",
            Hardware.liftingEncoder.getDistance());

    // System.out.println(
    // "Lift Encoder Ticks " + Hardware.liftingEncoder.get());
    SmartDashboard.putNumber("Lift Encoder Ticks",
            Hardware.liftingEncoder.get());

    // System.out.println("Intake Deploy Encoder "
    // + Hardware.intakeDeployEncoder.getDistance());
    SmartDashboard.putNumber("Intake Deploy Encoder",
            Hardware.intakeDeployEncoder.getDistance());

    // System.out.println("Intake Deploy Encoder Ticks "
    // + Hardware.intakeDeployEncoder.get());
    SmartDashboard.putNumber("Intake Deploy Ticks",
            Hardware.intakeDeployEncoder.get());

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
    SmartDashboard.putBoolean("Photo SW",
            Hardware.cubePhotoSwitch.isOn());

    // =================================
    // Pneumatics
    // =================================

    // ---------------------------------
    // Compressor
    // prints information on the compressor
    // ---------------------------------

    // ---------------------------------
    // Solenoids
    // ---------------------------------

    // Analogs
    // =================================

    // ---------------------------------
    // pots
    // where the pot is turned to
    // ---------------------------------
    // System.out
    // .println("Delay Pot " + Hardware.delayPot.get(0, 5));
    SmartDashboard.putNumber("Delay Pot",
            Hardware.delayPot.get(0, 5));

    // ---------------------------------
    // GYRO
    // ---------------------------------

    // System.out.println("AnalogGyro: " + Hardware.gyroAnalog.getAngle());
    // SmartDashboard.putNumber("AnalogGyro",
    // Hardware.gyroAnalog.getAngle());

    // System.out.println("Gyro: " + Hardware.gyro.getAngle());
    // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());


    // ---------------------------------
    // Sonar/UltraSonic
    // ---------------------------------
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

    // =================================
    // SPI Bus
    // =================================

    // -------------------------------------
    // Analog Interfaces
    // -------------------------------------

    // System.out.println("Gyro: " + Hardware.gyro.getAngle());
    // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());

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
