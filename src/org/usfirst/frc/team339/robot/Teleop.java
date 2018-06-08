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

import com.ctre.phoenix.motorcontrol.ControlMode;
import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.DrivePID;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.DrivePID.PIDDriveFunction;
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
 * User Initialization code for teleop mode should go here. Will be called once
 * when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */

public static void init ()
{


    // User code goes below here
    // temporary timer
    Hardware.telemetry.printToShuffleboard();
    Hardware.testTimer.start();
    Hardware.telemetry.setTimeBetweenPrints(10000);
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

    // ---------------------------------
    // setup motors
    // ---------------------------------
    Hardware.transmission.setForTeleop(Robot.KILROY_XIX_GEAR_2_SPEED);
    Hardware.rightDriveMotor.set(0);
    Hardware.leftDriveMotor.set(0);

    SmartDashboard.putNumber("Turn Power", 0);
    // SmartDashboard.putNumber("Turn Brake Power", 0);
    // SmartDashboard.putNumber("Turn Brake Deadband", 0);
    // SmartDashboard.putNumber("Drive Power", 0);
    // SmartDashboard.putNumber("Drive Straight Constant", 0);
    // SmartDashboard.putNumber("Drive Brake Power", 0);
    // SmartDashboard.putNumber("Drive Brake Deadband", 0);
    //
    SmartDashboard.putNumber("Turn Degrees", 0);
    drivepid.tunePID(PIDDriveFunction.TURN);
    // SmartDashboard.putNumber("Drive Distance", 0);

    if (Hardware.demoModeSwitch.isOn() == true)
        {
        Hardware.transmission.setGearPercentage(0,
                Hardware.delayPot.get(0, .7));
        Hardware.transmission.setGear(0);
        }

} // end Init

// tune pid loop

// private static boolean hasSeenTape = false;

/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void periodic ()
{
    // Hardware.tempRelay.set(true);

    // if (Hardware.redLight.isOn() && hasSeenTape == false)
    // {
    // hasSeenTape = true;
    // System.out.println("RL Has Seen Tape! Yay!");
    // }

    // Stress testing... TAKE OUT!
    // if (Hardware.leftOperator.getRawButton(8) == true)
    // {
    // Hardware.tempRelay.set(true);
    // Hardware.axisCamera.processImage();
    // Hardware.axisCamera.processImage();
    // Hardware.axisCamera.processImage();
    // }

    // =================================================================
    // OPERATOR CONTROLS
    // =================================================================

    // code for alignToScale; this is the best current version as of 8 p.m.
    // on
    // Monday
    // TODO currently untested
    // Hardware.scaleAlignment.alignToScaleByButtons(
    // Hardware.leftOperator.getRawButton(4));



    if (Hardware.demoModeSwitch.isOn() == false)
        {
        // We are in COMPETITION MODE!!!
        Hardware.cubeManipulator.intakeCube(
                Hardware.rightOperator.getTrigger(),
                Hardware.rightOperator.getRawButton(2));
        Hardware.cubeManipulator.ejectCube(
                Hardware.leftOperator.getTrigger(),
                (Hardware.leftOperator.getRawButton(3)));
        }
    else
        {
        // We are in DEMO MODE!!!
        Hardware.cubeManipulator
                .intakeCube(Hardware.rightOperator.getTrigger(), false);
        Hardware.cubeManipulator.ejectCube(false,
                Hardware.rightOperator.getRawButton(2));
        }

    // -----------------------------------------
    // Deploy Intake controls
    // -----------------------------------------
    // Button 11 to deploy, 10 to retract, and 9 for override for both.
    if (Hardware.demoModeSwitch.isOn() == false)
        {
        // We are in COMPETITION MODE!!!
        if (Hardware.leftOperator.getRawButton(11))
            Hardware.cubeManipulator.deployCubeIntake(
                    Hardware.leftOperator.getRawButton(9));
        else if (Hardware.leftOperator.getRawButton(10))
            Hardware.cubeManipulator.retractCubeIntake(
                    Hardware.leftOperator.getRawButton(9));
        else if (Hardware.rightOperator.getRawButton(11))
            Hardware.cubeManipulator.setDeployForClimb();

        if (Hardware.leftOperator.getRawButton(2))
            Hardware.cubeManipulator.angleDeployForScale();
        }

    // -----------------------------------------
    // Forklift (not Cube Manipulator) controls
    // -----------------------------------------

    // --------------------------------------------------------------
    // CAN TESTING CODE
    // --------------------------------------------------------------


    // tank code
    if (Hardware.leftDriver.getY() > .2)
        leftInput = Hardware.leftDriver.getY() * .8 - .2;
    else if (Hardware.leftDriver.getY() < -.2)
        leftInput = Hardware.leftDriver.getY() * .8 + .2;
    else
        leftInput = 0.0;

    if (Hardware.rightDriver.getY() > .2)
        rightInput = Hardware.rightDriver.getY() * .8 - .2;
    else if (Hardware.rightDriver.getY() < -.2)
        rightInput = Hardware.rightDriver.getY() * .8 + .2;
    else
        rightInput = 0.0;

    rightInput *= .4;
    leftInput *= .4;


    Hardware.rightRearCANMotor.follow(Hardware.rightFrontCANMotor);
    Hardware.leftRearCANMotor.follow(Hardware.leftFrontCANMotor);


    // Hardware.leftRearCANMotor.set(ControlMode.PercentOutput,
    // Hardware.leftOperator.getY());
    //
    // Hardware.rightRearCANMotor.set(ControlMode.PercentOutput,
    // Hardware.rightOperator.getY());

    Hardware.rightFrontCANMotor.set(ControlMode.PercentOutput,
            rightInput);

    Hardware.leftFrontCANMotor.set(ControlMode.PercentOutput,
            leftInput);
    // System.out.println("CAN " +
    // Hardware.rightFrontCANMotor.getMotorOutputPercent());



    // --------------------------------------------------------------
    if (Hardware.demoModeSwitch.isOn() == false)
        // We are in COMPETITION MODE!!!
        Hardware.cubeManipulator.moveForkliftWithController(
                Hardware.rightOperator.getY(),
                Hardware.rightOperator.getRawButton(5),
                Hardware.climbButton.isOnCheckNow());
    else
        // We are in DEMO MODE!!!
        Hardware.cubeManipulator.moveForkliftWithController(
                Hardware.rightOperator.getY() * Robot.demoForkliftSpeed,
                false, false);

    if (Hardware.demoModeSwitch.isOn() == false)
        {
        // We are in COMPETITION MODE!!!
        if (Hardware.rightOperator.getRawButton(6) == true)
            Hardware.cubeManipulator
                    .setLiftPosition(CubeManipulator.SCALE_HEIGHT, .6);
        else if (Hardware.rightOperator.getRawButton(7) == true)
            Hardware.cubeManipulator
                    .setLiftPosition(CubeManipulator.SWITCH_HEIGHT, .6);

        if (Hardware.climbButton.isOnCheckNow() == true)
            Hardware.climbingMechanismServo
                    .set(Robot.CLIMB_SERVO_CLIMB_POISITION);
        else
            Hardware.climbingMechanismServo
                    .set(Robot.CLIMB_SERVO_INIT_POSITION);
        }

    // update for the cube manipulator (forklift, intake, etc.) and its
    // state
    // machines
    Hardware.cubeManipulator.masterUpdate();

    // =================================================================
    // CAMERA CODE
    // =================================================================

    if (Hardware.demoModeSwitch.isOn() == false)
        // We are in COMPETITION MODE!!!
        // TODO @ANE uncomment
        // Hardware.axisCamera
        // .takeLitPicture(Hardware.leftOperator.getRawButton(6)
        // && Hardware.leftOperator.getRawButton(7));

        // =================================================================
        // DRIVING CODE
        // =================================================================
        // if the right driver button 3 is pressed, shift up a gear, if the left
        // driver button 3 id pressed, shift down a gear
        if (Hardware.demoModeSwitch.isOn() == false)
            // We are in COMPETITION MODE!!!
            Hardware.transmission.shiftGears(
                    Hardware.rightDriver.getRawButton(3),
                    Hardware.leftDriver.getRawButton(3));
        else
            {
            // We are in DEMO MODE!!!
            if (Hardware.leftDriver.getRawButton(7) == true)
                Hardware.transmission.setGearPercentage(0, .6);
            else if (Hardware.leftDriver.getRawButton(8) == true)
                Hardware.transmission.setGearPercentage(0,
                        Hardware.delayPot.get(0, .6));
            else if (Hardware.leftDriver.getRawButton(9) == true)
                Hardware.transmission.setGearPercentage(0, .4);

            }

    // if is testing drive is equal to true, the joysticks are locked out to
    // test some sort of drive function (of drive by camera)
    //
    // if (isTestingDrive == false
    // && Hardware.scaleAlignment.allowAlignment == false
    // && isBeckyTest == false
    // && isTestingEncoderTurn == false)
    if (isTestingDrive == false)
        Hardware.transmission.drive(Hardware.leftDriver,
                Hardware.rightDriver);
    // update

    // ------------------------------------
    // print out any information needed to
    // display on the drivers station
    // ------------------------------------

    // printStatements();
    Hardware.telemetry.printToConsole();

    // temporary timer
    if (Hardware.testTimer.get() >= 30
            && Hardware.testTimerFlag == false)
        {
        Hardware.telemetry.setTimeBetweenPrints(1000);
        Hardware.testTimer.reset();


        Hardware.testTimerFlag = true;

        }
    if (Hardware.testTimer.get() >= 30
            && Hardware.testTimerFlag == true)
        {
        Hardware.telemetry.setTimeBetweenPrints(5000);
        Hardware.testTimer.reset();

        Hardware.testTimerFlag = false;
        }

    // -------------------------------------------
    // Put anything you need to test, but the
    // code will not be a part of the final teleop
    // -------------------------------------------

    // Hardware.intakeArmPositionServo
    // .set(Hardware.leftDriver.getThrottle());
    // System.out.println("Servo: " + Hardware.leftDriver.getThrottle());

    if (Hardware.demoModeSwitch.isOn() == false)
        {
        testingDrive();
        // liftTest();
        // beckyTest();
        }
} // end Periodic()

private static boolean allowAlignment = false;

private static boolean isTestingDrive = false;

private static boolean isTestingEncoderTurn = false;

private static int driveState = 0;

public static void alignScale ()
{

}

private static boolean isBeckyTest = false;

// private static void beckyTest ()
// {
// // Hardware.ringLightRelay.set(Value.kForward);
// if (Hardware.rightOperator.getRawButton(2) == true)
// {
// Hardware.transmission.setForAutonomous();
// isBeckyTest = true;
// }
//
// if (isBeckyTest == true)
// {
//
// if (Hardware.driveWithCamera.driveToSwitch(.3) == true)
// {
// Hardware.transmission
// .setForTeleop(Robot.KILROY_XV_GEAR_2_SPEED);
// isBeckyTest = false;
// }
// Hardware.tempRelay.set(true);
// // Hardware.tempRelay.set(true);
// Hardware.axisCamera.saveImage(ImageType.PROCESSED);
// }
//
// // if (Hardware.rightOperator.getRawButton(2))
// // {
// // Hardware.tempRelay.set(true);
// // }
// // else
// // {
// // Hardware.tempRelay.set(false);
// // }
// //
// // if(Hardware.rightOperator.getRawButton(3))
// // {
// // Hardware.ringLightRelay.setDirection(Direction.kForward);
// // }
// // else
// // {
// // Hardware.ringLightRelay.setDirection(Direction.kReverse);
// // }
//
// // if (Hardware.leftOperator.getRawButton(7))
// // {
// // Hardware.ringLightRelay.set(Value.kForward);
// // }
// // else
// // {
// // Hardware.ringLightRelay.set(Value.kReverse);
// // }
//
// // Hardware.axisCamera.saveImage(ImageType.RAW);
//
// // Hardware.driveWithCamera.getCameraCenterValue();
//
// // if (Hardware.visionTestButton.isOnCheckNow())
// // {
// // Hardware.axisCamera.processImage();
// // Hardware.autoDrive.visionTest(1.3, .6);
// // Hardware.axisCamera.saveImage(ImageType.PROCESSED);
// // for (int i = 0; i < Hardware.axisCamera
// // .getParticleReports().length; i++)
// // {
// // System.out.println("The center of " + i + " is: "
// // + Hardware.axisCamera.getNthSizeBlob(i).center.x);
// // }
// // System.out.println("The center is : " + (Hardware.axisCamera
// // .getNthSizeBlob(0).center.x
// // + Hardware.axisCamera.getNthSizeBlob(1).center.x) / 2);
// // }
// } // end beckyTest()

static DrivePID drivepid = new DrivePID(Hardware.transmission,
        Hardware.leftFrontDriveEncoder,
        Hardware.rightFrontDriveEncoder, Hardware.gyro);

private static void testingDrive ()
{
    if (Hardware.rightDriver.getRawButton(10) == true)
        Hardware.autoDrive.resetEncoders();

    if (Hardware.rightDriver.getRawButton(9) == true)
        drivepid.tunePID(PIDDriveFunction.TURN);

    if (Hardware.rightDriver.getRawButton(7) == true)
        isTestingDrive = true;

    if (isTestingDrive == true)
        {
        if (Hardware.rightDriver.getRawButton(8) == true
                || drivepid.turnDegrees(
                        (int) SmartDashboard.getNumber("Turn Degrees",
                                0),
                        SmartDashboard.getNumber("Turn Power", 0)))
            {
            drivepid.reset();
            isTestingDrive = false;
            }
        }

} // end of testingDrive()

private static void liftTest ()
{
    // SmartDashboard.putNumber("throttle value : ",
    // Hardware.leftOperator.getThrottle());

    // Hardware.liftingMotor
    // .set(SmartDashboard.getNumber("Lifting motor speed", 0));
    //
    // Hardware.cubeIntakeMotor
    // .set(SmartDashboard.getNumber("Cube intake speed", 0));
    //
    // Hardware.cubeIntakeMotor
    // .set(SmartDashboard.getNumber("Intake Speed: ", 0));
}

/**
 * stores print statements for future use in the print "bank", statements are
 * commented out when not in use, when you write a new print statement,
 * "deposit" the statement in the correct "bank" do not "withdraw" statements,
 * unless directed to.
 * 
 * NOTE: Keep the groupings below, which correspond in number and order as the
 * hardware declarations in the HARDWARE class
 * 
 * @author Ashley Espeland
 * @written 1/28/16
 * 
 *          Edited by Ryan McGee Also Edited by Josef Liebl
 * 
 */
public static void printStatements ()
{
    System.out.println("pdp 0 = " + Hardware.pdp.getCurrent(0));
    System.out.println("pdp 1 = " + Hardware.pdp.getCurrent(1));
    System.out.println("pdp 2 = " + Hardware.pdp.getCurrent(2) + "\n");

    System.out.println("pdp 3 = " + Hardware.pdp.getCurrent(3));
    System.out.println("pdp 4 = " + Hardware.pdp.getCurrent(4));
    System.out.println("pdp 5 = " + Hardware.pdp.getCurrent(5) + "\n");

    System.out.println("pdp 6 = " + Hardware.pdp.getCurrent(6));
    System.out.println("pdp 7 = " + Hardware.pdp.getCurrent(7));
    System.out.println("pdp 8 = " + Hardware.pdp.getCurrent(8));



    if (Hardware.driverStation.isFMSAttached() == false)
        {
        // ==================================
        // Scale Alignment
        // ==================================
        // SmartDashboard.putString("Relative to scale",
        // Hardware.scaleAlignment.RelativeScale);

        // SmartDashboard.putNumber("RearUltraSonic",
        // Hardware.rearUltraSonic.getDistanceFromNearestBumper());

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
        // {
        // System.out.println(
        // "Disable = off");
        // }
        // else
        // {
        // System.out.println(
        // "Disable = on");
        // }
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


        // System.out.println("6 pos = "
        // + Hardware.autoSixPosSwitch.getPosition());
        SmartDashboard.putNumber("6 Pos Switch",
                Hardware.autoSixPosSwitch.getPosition());

        SmartDashboard.putBoolean("Demo Switch",
                Hardware.demoModeSwitch.isOn());

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
        // System.out.println("Left Red Light " +
        // Hardware.leftRedLight.isOn());
        // SmartDashboard.putBoolean("L Red Light",
        // Hardware.leftRedLight.isOn());
        // System.out.println(
        // "PhotoSwitch " + Hardware.cubePhotoSwitch.isOn());
        SmartDashboard.putBoolean("Photo SW",
                Hardware.cubePhotoSwitch.isOn());

        SmartDashboard.putBoolean("IR is On", Hardware.armIR.isOn());

        SmartDashboard.putBoolean("Bottom RL: ",
                Hardware.redLight.isOn());


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
        // SmartDashboard.putNumber("Delay Pot",
        // Hardware.delayPot.get(0, 5));

        // ---------------------------------
        // GYRO
        // ---------------------------------

        // System.out.println("AnalogGyro: " +
        // Hardware.gyroAnalog.getAngle());
        SmartDashboard.putNumber("AnalogGyro",
                Hardware.gyroAnalog.getAngle());

        // System.out.println("Gyro: " + Hardware.gyro.getAngle());
        // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());


        // ---------------------------------
        // Sonar/UltraSonic
        // ---------------------------------
        // SmartDashboard.putNumber("Rear Ultrasonic distance",
        // Hardware.rearUltraSonic.getDistanceFromNearestBumper());

        // System.out.println("Front UltraSonic "
        // + Hardware.frontUltraSonic.getDistanceFromNearestBumper());
        SmartDashboard.putNumber("Front Ultrasonic",
                Hardware.frontUltraSonic
                        .getDistanceFromNearestBumper());
        //
        // SmartDashboard.putNumber("Front Ultrasonic Raw",
        // Hardware.frontUltraSonic.getRefinedDistanceValue());
        //
        // SmartDashboard.putNumber("Front ultrasonic bumper",
        // Hardware.frontUltraSonic
        // .getOffsetDistanceFromNearestBumper());
        // System.out.println("Rear UltraSonic "
        // + Hardware.rearUltraSonic.getDistanceFromNearestBumper());
        // SmartDashboard.putNumber("Rear Ultrasonic",
        // Hardware.rearUltraSonic.getDistanceFromNearestBumper());
        // SmartDashboard.putNumber("Rear Ultrasonic Raw",
        // Hardware.rearUltraSonic.getRefinedDistanceValue());

        // =========================
        // Servos
        // =========================
        // System.out.println("Climbing Mechanism Servo" +
        // Hardware.climbingMechanismServo.getAngle());
        // SmartDashboard.putNumber("Climb Servo",
        // Hardware.climbingMechanismServo.getAngle());

        // System.out.println("Intake Arm Servo " +
        // Hardware.intakeArmPositionServo.getAngle());

        // =================================
        // SPI Bus
        // =================================

        // -------------------------------------
        // Analog Interfaces
        // -------------------------------------

        // System.out.println("Gyro: " + Hardware.gyro.getAngle());
        SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());

        // =================================
        // Connection Items
        // =================================

        // ---------------------------------
        // Cameras
        // prints any camera information required
        // ---------------------------------
        // System.out.println("The camera center is: "
        // + Hardware.driveWithCamera.getCameraCenterValue());
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

        // ---------------------------------
        // Final SmartDasboard
        // code for values we want on the final dashboard layout we use
        // during
        // matches
        // ---------------------------------

        // set of if elses to properly set the two boolean boxes on the
        // SmartDashboard that are used to track the status of the cube. On
        // the
        // actual SmartDashboard, the boolean boxes are layered so between
        // the
        // two
        // of them, the layered box is green when we have a cube, red when
        // we
        // are
        // intaking a cube, and transparent if we don't have a cube and are
        // not
        // intaking

        }

    if (Hardware.cubeManipulator.hasCube() == true)
        {
        // SmartDashboard value that is used to tell whether or not we have
        // a
        // cube; true sets the boolean box to green; false sets the boolean
        // box
        // to transparent
        SmartDashboard.putBoolean("Has Cube", true);
        // SmartDashboard value that tracks if the intake is intaking; has
        // the
        // the same key as the cube one, except with a space at the end, for
        // aesthetic reasons when it shows up on the SmartDashboard; true
        // sets
        // the box to red; false sets the box to transparent
        SmartDashboard.putBoolean("Has Cube ", false);

        // }
        // else if (Hardware.cubeManipulator
        // .getIntakeMotorSpeed() >
        // Hardware.cubeManipulator.INTAKE_STOP_WITH_CUBE
        // + .1
        /*
         * the .1 here is just a magic number that
         * wasn't worth making a constant for; is meant
         * to prevent false positives in cases where
         * the getIntakeMotorSpeed is returning
         * .2000001 or something
         */
        // )
        // {
        // sets the boolean box to transparent
        SmartDashboard.putBoolean("Has Cube", false);
        // sets the other boolean box to red
        SmartDashboard.putBoolean("Has Cube ", true);
        }
    else
        {
        // sets the boolean box to transparent
        SmartDashboard.putBoolean("Has Cube", false);
        // sets the other boolean box to transparent
        SmartDashboard.putBoolean("Has Cube ", false);
        }


    SmartDashboard.putBoolean("Too close to scale",
            Hardware.armIR.isOn());

    SmartDashboard.updateValues();
} // end printStatements()

// ================================
// Constants
// ================================
public static final int CLIMBING_SERVO_ANGLE = 78;

public static final int INTAKE_ARM_SERVO_UP_POSITION = 0;

public static final int INTAKE_ARM_SERVO_DOWN_POSITION = 180;

public static final double FORKLIFT_HEIGHT_TO_PUT_DOWN_SERVO = 20.0;

// ================================
// Variables
// ================================
public static double rightInput = 0.0;

public static double leftInput = 0.0;


} // end class
