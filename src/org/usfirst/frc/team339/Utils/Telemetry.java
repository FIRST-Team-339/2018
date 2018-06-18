package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// class designated to all print statements to the console and shuffleboard

public class Telemetry
{

public static double lastTimePrinted = 0.0;

public static final double TIME_BETWEEN_PRINTS = 10000;


public static void printToConsole ()
{

    if (lastTimePrinted == 0)
        {
        lastTimePrinted = System.currentTimeMillis();
        }

    if (System.currentTimeMillis() - lastTimePrinted >= 1000)
        {


        if (Hardware.driverStation.isFMSAttached() == false)
            {
            // ==================================
            // Scale Alignment
            // ==================================


            // =================================
            // Motor
            // Prints the value of motors
            // =================================
            System.out.println(
                    "Right Drive Motor "
                            + Hardware.rightDriveMotor.get());

            System.out.println(
                    "Left Drive Motor "
                            + Hardware.leftDriveMotor.get());

            System.out.println("Lifting Motor " +
                    Hardware.liftingMotor.get());

            System.out.println(
                    "Cube Intake Motor "
                            + Hardware.cubeIntakeMotor.get());

            System.out.println(
                    "Intake Deploy Arm "
                            + Hardware.intakeDeployArm.get());

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



            // System.out.println("6 pos = "
            // + Hardware.autoSixPosSwitch.getPosition());

            // ---------------------------------
            // Encoders
            // ---------------------------------
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

            // System.out.println("Lift Encoder Inches = "
            // + Hardware.liftingEncoder.getDistance());

            // System.out.println(
            // "Lift Encoder Ticks " + Hardware.liftingEncoder.get());

            // System.out.println("Intake Deploy Encoder "
            // + Hardware.intakeDeployEncoder.getDistance());

            // System.out.println("Intake Deploy Encoder Ticks "
            // + Hardware.intakeDeployEncoder.get());

            // ---------------------------------
            // Red Light/IR Sensors
            // prints the state of the sensor
            // ---------------------------------
            // System.out
            // .println("Right Red Light " + Hardware.rightRedLight.isOn());

            // System.out.println("Left Red Light " +
            // Hardware.leftRedLight.isOn());

            // System.out.println(
            // "PhotoSwitch " + Hardware.cubePhotoSwitch.isOn());

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


            // ---------------------------------
            // GYRO
            // ---------------------------------

            // System.out.println("AnalogGyro: " +
            // Hardware.gyroAnalog.getAngle());

            // System.out.println("Gyro: " + Hardware.gyro.getAngle());
            // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());


            // ---------------------------------
            // Sonar/UltraSonic
            // ---------------------------------

            // System.out.println("Front UltraSonic "
            // + Hardware.frontUltraSonic.getDistanceFromNearestBumper());

            //


            // System.out.println("Rear UltraSonic "
            // + Hardware.rearUltraSonic.getDistanceFromNearestBumper());

            // =========================
            // Servos
            // =========================
            // System.out.println("Climbing Mechanism Servo" +
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

            // =================================
            // Connection Items
            // =================================

            // ---------------------------------
            // Cameras
            // prints any camera information required
            // ---------------------------------
            // System.out.println("The camera center is: "
            // + Hardware.driveWithCamera.getCameraCenterValue());

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
            // KILROY ANCILLARY ITEMS
            // =================================
            // ---------------------------------
            // Gear number displayed to driver
            // ---------------------------------
            // System.out.println(
            // "Gear = " + Hardware.transmission.getCurrentGear() + 1);


            // ---------------------------------
            // timers
            // what time does the timer have now
            // ---------------------------------



            }


        }

    lastTimePrinted = System.currentTimeMillis();
}

public static void printToShuffleboard ()
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
    SmartDashboard.putNumber("R Drive Motor",
            Hardware.rightDriveMotor.get());

    SmartDashboard.putNumber("L Drive Motor",
            Hardware.leftDriveMotor.get());


    // SmartDashboard.putNumber("Lifting Motor",
    // Hardware.liftingMotor.get());


    SmartDashboard.putNumber("Cube Motor",
            Hardware.cubeIntakeMotor.get());

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

    // =================================
    // // Digital Inputs
    // =================================
    //
    // ---------------------------------
    // Switches
    // prints state of switches
    // ---------------------------------

    // SmartDashboard.putBoolean("Disable SW",
    // Hardware.disableAutonomousSwitch.isOn());

    // SmartDashboard.putBoolean("L Auto SW",
    // Hardware.leftAutoSwitch.isOn());
    //
    //
    // SmartDashboard.putBoolean("R Auto SW",
    // Hardware.rightAutoSwitch.isOn());

    // SmartDashboard.putNumber("6 Pos Switch",
    // Hardware.autoSixPosSwitch.getPosition());

    // ---------------------------------
    // Encoders
    // ---------------------------------
    SmartDashboard.putNumber("Left Front Encoder Inches",
            Hardware.leftFrontDriveEncoder.getDistance());

    SmartDashboard.putNumber("Left Front Encoder Ticks",
            Hardware.leftFrontDriveEncoder.get());


    SmartDashboard.putNumber("Right Front Encoder Inches",
            Hardware.rightFrontDriveEncoder.getDistance());


    SmartDashboard.putNumber("Right Front Encoder Ticks",
            Hardware.rightFrontDriveEncoder.get());

    SmartDashboard.putNumber("Left Rear Encoder Inches",
            Hardware.leftRearDriveEncoder.getDistance());


    SmartDashboard.putNumber("Left Rear Encoder Ticks",
            Hardware.leftRearDriveEncoder.get());

    SmartDashboard.putNumber("Right Rear Encoder Inches",
            Hardware.rightRearDriveEncoder.getDistance());

    SmartDashboard.putNumber("Right Rear Encoder Ticks",
            Hardware.rightRearDriveEncoder.get());

    SmartDashboard.putNumber("Lift Encoder Inches",
            Hardware.liftingEncoder.getDistance());

    SmartDashboard.putNumber("Lift Encoder Ticks",
            Hardware.liftingEncoder.get());

    SmartDashboard.putNumber("Intake Deploy Encoder",
            Hardware.intakeDeployEncoder.getDistance());

    SmartDashboard.putNumber("Intake Deploy Ticks",
            Hardware.intakeDeployEncoder.get());

    // ---------------------------------
    // Red Light/IR Sensors
    // prints the state of the sensor
    // ---------------------------------

    // SmartDashboard.putBoolean("R Red Light",
    // Hardware.rightRedLight.isOn());

    // SmartDashboard.putBoolean("L Red Light",
    // Hardware.leftRedLight.isOn());

    SmartDashboard.putBoolean("Photo SW",
            Hardware.cubePhotoSwitch.isOn());

    SmartDashboard.putBoolean("IR is On",
            Hardware.armIR.isOn());

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

    // SmartDashboard.putNumber("Delay Pot",
    // Hardware.delayPot.get(0, 5));

    // ---------------------------------
    // GYRO
    // ---------------------------------


    SmartDashboard.putNumber("AnalogGyro",
            Hardware.gyroAnalog.getAngle());

    // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());


    // ---------------------------------
    // Sonar/UltraSonic
    // ---------------------------------
    // SmartDashboard.putNumber("Rear Ultrasonic distance",
    // Hardware.rearUltraSonic.getDistanceFromNearestBumper());


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

    // SmartDashboard.putNumber("Rear Ultrasonic",
    // Hardware.rearUltraSonic.getDistanceFromNearestBumper());
    // SmartDashboard.putNumber("Rear Ultrasonic Raw",
    // Hardware.rearUltraSonic.getRefinedDistanceValue());

    // =========================
    // Servos
    // =========================
    //
    // SmartDashboard.putNumber("Climb Servo",
    // Hardware.climbingMechanismServo.getAngle());

    //
    // =================================
    // SPI Bus
    // =================================

    // -------------------------------------
    // Analog Interfaces
    // -------------------------------------

    SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());

    // =================================
    // Connection Items
    // =================================

    // ---------------------------------
    // Cameras
    // prints any camera information required
    // ---------------------------------

    SmartDashboard.putNumber("Camera Center",
            Hardware.driveWithCamera.getCameraCenterValue());

    // =================================
    // Driver station
    // =================================

    // ---------------------------------
    // Joysticks
    // information about the joysticks
    // ---------------------------------

    // SmartDashboard.putNumber("R Driver Y Joy",
    // Hardware.rightDriver.getY());

    // SmartDashboard.putNumber("L Driver Y Joy",
    // Hardware.leftDriver.getY());

    // SmartDashboard.putNumber("R Operator Y Joy",
    // Hardware.rightOperator.getY());

    // SmartDashboard.putNumber("L Operator Y Joy",
    // Hardware.leftOperator.getY());

    // =================================
    // KILROY ANCILLARY ITEMS
    // =================================
    // ---------------------------------
    // Gear number displayed to driver
    // ---------------------------------
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

        }
    else if (Hardware.cubeManipulator
            .getIntakeMotorSpeed() > Hardware.cubeManipulator.INTAKE_STOP_WITH_CUBE
                    + .1 /*
                          * the
                          * .1
                          * here
                          * is
                          * just
                          * a
                          * magic
                          * number
                          * that
                          * wasn
                          * '
                          * t
                          * worth
                          * making
                          * a
                          * constant
                          * for;
                          * is
                          * meant
                          * to
                          * prevent
                          * false
                          * positives
                          * in
                          * cases
                          * where
                          * the
                          * getIntakeMotorSpeed
                          * is
                          * returning
                          * .2000001
                          * or
                          * something
                          */)
        {
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
}





}