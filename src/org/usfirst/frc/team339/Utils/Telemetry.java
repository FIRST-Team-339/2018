package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dedicated to printing information to both
 * Console and Smartdashboard
 * 
 * @author Craig Kimball
 * @written 6/7/2018
 *
 */

public class Telemetry
{
/**
 * constructor for
 * 
 */
public Telemetry ()
{

    this.init();
}

/**
 * creates the Telemetry object. Used for the set function of the
 * timeBetweenPrints
 * 
 * @param newTimeBetweenPrints
 */
public Telemetry (double newTimeBetweenPrints)
{
    this.setTimeBetweenPrints(newTimeBetweenPrints);
    this.init();
}

/**
 * Returns last time print statements were printed
 * 
 * @return lastTimePrinted
 */
private double getLastTimePrinted ()
{
    return (lastTimePrinted);
}

/**
 * function dedicated to returning the current time between prints
 * 
 * @return timeBetweenPrints
 */
public double getTimeBetweenPrints ()
{
    return (timeBetweenPrints);

}

/**
 * Initializes telemetry object by setting lastTimePrinted to the system's
 * current time
 */
private void init ()
{

    // getting the initial time
    lastTimePrinted = System.currentTimeMillis();

}

/**
 * @param newLastTimePrinted
 *            sets lastTimePrinted to what you want
 * @return
 */
private double setLastTimePrinted (double newLastTimePrinted)
{
    lastTimePrinted = newLastTimePrinted;
    return (this.getLastTimePrinted());
}

/**
 * function dedicated to setting a new value for time between each
 * print
 * 
 * @param newTimeBetweenPrints
 * @return
 */
public double setTimeBetweenPrints (double newTimeBetweenPrints)
{
    timeBetweenPrints = newTimeBetweenPrints;
    return (this.getTimeBetweenPrints());
}

/**
 * prints to console
 * 
 */
public void printToConsole ()
{
    // clock determining change

    if ((System.currentTimeMillis()
            - lastTimePrinted) >= this.getTimeBetweenPrints())
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
                            + Hardware.rightFrontCANMotor.get());

            System.out.println(
                    "Left Drive Motor "
                            + Hardware.leftFrontCANMotor.get());

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
            // System.out.println("voltage " + Hardware.pdp.getVoltage());
            // System.out.println("total current " +
            // Hardware.pdp.getTotalCurrent());
            //
            // System.out.println("pdp 0 = " + Hardware.pdp.getCurrent(0));
            // System.out.println("pdp 1 = " + Hardware.pdp.getCurrent(1));
            // System.out.println("pdp 2 = " + Hardware.pdp.getCurrent(2) +
            // "\n");
            //
            // System.out.println("pdp 3 = " + Hardware.pdp.getCurrent(3));
            // System.out.println("pdp 12 = " + Hardware.pdp.getCurrent(12));
            // System.out.println("pdp 13 = " + Hardware.pdp.getCurrent(13) +
            // "\n");
            //
            // System.out.println("pdp 14 = " + Hardware.pdp.getCurrent(14));
            // System.out.println("pdp 15 = " + Hardware.pdp.getCurrent(15));
            // System.out.println("pdp 13 = " + Hardware.pdp.getCurrent(13));
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

            System.out.println(
                    "Demo swich isOn" + Hardware.demoModeSwitch.isOn());

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
            System.out.println("Climbing Mechanism Servo" +
                    Hardware.climbingMechanismServo.getAngle());

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

        this.setLastTimePrinted(System.currentTimeMillis());
        }
    // resets the clock to 0

}

/**
 * prints to shuffleboard / smartboard
 * 
 */
public void printToShuffleboard ()
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
            Hardware.rightFrontCANMotor.get());

    SmartDashboard.putNumber("L Drive Motor",
            Hardware.leftFrontCANMotor.get());


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
    // // ==================================
    // // Scale Alignment
    // // ==================================
    // // SmartDashboard.putString("Relative to scale",
    // // Hardware.scaleAlignment.RelativeScale);
    //
    // // SmartDashboard.putNumber("RearUltraSonic",
    // // Hardware.rearUltraSonic.getDistanceFromNearestBumper());
    //
    // // =================================
    // // Motor
    // // Prints the value of motors
    // // =================================
    // SmartDashboard.putNumber("R Drive Motor",
    // Hardware.rightDriveMotor.get());
    //
    // SmartDashboard.putNumber("L Drive Motor",
    // Hardware.leftDriveMotor.get());
    //
    //
    // // SmartDashboard.putNumber("Lifting Motor",
    // // Hardware.liftingMotor.get());
    //
    //
    // SmartDashboard.putNumber("Cube Motor",
    // Hardware.cubeIntakeMotor.get());
    //
    // // SmartDashboard.putNumber("Intake Deploy Motor",
    // // Hardware.intakeDeployArm.get());
    // // =================================
    // // CAN items
    // // prints value of the CAN controllers
    // // =================================
    // //
    // // =================================
    // // Relay
    // // =================================
    //
    // // =================================
    // // // Digital Inputs
    // // =================================
    // //
    // // ---------------------------------
    // // Switches
    // // prints state of switches
    // // ---------------------------------
    //
    // // SmartDashboard.putBoolean("Disable SW",
    // // Hardware.disableAutonomousSwitch.isOn());
    //
    // // SmartDashboard.putBoolean("L Auto SW",
    // // Hardware.leftAutoSwitch.isOn());
    // //
    // //
    // // SmartDashboard.putBoolean("R Auto SW",
    // // Hardware.rightAutoSwitch.isOn());
    //
    SmartDashboard.putNumber("6 Pos Switch",
            Hardware.autoSixPosSwitch.getPosition());
    //
    // // ---------------------------------
    // // Encoders
    // // ---------------------------------
    // SmartDashboard.putNumber("Left Front Encoder Inches",
    // Hardware.leftFrontDriveEncoder.getDistance());
    //
    // SmartDashboard.putNumber("Left Front Encoder Ticks",
    // Hardware.leftFrontDriveEncoder.get());
    //
    //
    // SmartDashboard.putNumber("Right Front Encoder Inches",
    // Hardware.rightFrontDriveEncoder.getDistance());
    //
    //
    // SmartDashboard.putNumber("Right Front Encoder Ticks",
    // Hardware.rightFrontDriveEncoder.get());
    //
    // SmartDashboard.putNumber("Left Rear Encoder Inches",
    // Hardware.leftRearDriveEncoder.getDistance());
    //
    //
    // SmartDashboard.putNumber("Left Rear Encoder Ticks",
    // Hardware.leftRearDriveEncoder.get());
    //
    // SmartDashboard.putNumber("Right Rear Encoder Inches",
    // Hardware.rightRearDriveEncoder.getDistance());
    //
    // SmartDashboard.putNumber("Right Rear Encoder Ticks",
    // Hardware.rightRearDriveEncoder.get());
    //
    // SmartDashboard.putNumber("Lift Encoder Inches",
    // Hardware.liftingEncoder.getDistance());
    //
    // SmartDashboard.putNumber("Lift Encoder Ticks",
    // Hardware.liftingEncoder.get());
    //
    // SmartDashboard.putNumber("Intake Deploy Encoder",
    // Hardware.intakeDeployEncoder.getDistance());
    //
    // SmartDashboard.putNumber("Intake Deploy Ticks",
    // Hardware.intakeDeployEncoder.get());
    //
    // // ---------------------------------
    // // Red Light/IR Sensors
    // // prints the state of the sensor
    // // ---------------------------------
    //
    // // SmartDashboard.putBoolean("R Red Light",
    // // Hardware.rightRedLight.isOn());
    //
    // // SmartDashboard.putBoolean("L Red Light",
    // // Hardware.leftRedLight.isOn());
    //
    // SmartDashboard.putBoolean("Photo SW",
    // Hardware.cubePhotoSwitch.isOn());
    //
    // SmartDashboard.putBoolean("IR is On",
    // Hardware.armIR.isOn());
    //
    // SmartDashboard.putBoolean("Bottom RL: ",
    // Hardware.redLight.isOn());
    //
    //
    // // =================================
    // // Pneumatics
    // // =================================
    //
    // // ---------------------------------
    // // Compressor
    // // prints information on the compressor
    // // ---------------------------------
    //
    // // ---------------------------------
    // // Solenoids
    // // ---------------------------------
    //
    // // Analogs
    // // =================================
    //
    // // ---------------------------------
    // // pots
    // // where the pot is turned to
    // // ---------------------------------
    //
    // // SmartDashboard.putNumber("Delay Pot",
    // // Hardware.delayPot.get(0, 5));
    //
    // // ---------------------------------
    // // GYRO
    // // ---------------------------------
    //
    //
    // SmartDashboard.putNumber("AnalogGyro",
    // Hardware.gyroAnalog.getAngle());
    //
    // // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());
    //
    //
    // // ---------------------------------
    // // Sonar/UltraSonic
    // // ---------------------------------
    // // SmartDashboard.putNumber("Rear Ultrasonic distance",
    // // Hardware.rearUltraSonic.getDistanceFromNearestBumper());
    //
    //
    // SmartDashboard.putNumber("Front Ultrasonic",
    // Hardware.frontUltraSonic
    // .getDistanceFromNearestBumper());
    // //
    // // SmartDashboard.putNumber("Front Ultrasonic Raw",
    // // Hardware.frontUltraSonic.getRefinedDistanceValue());
    // //
    // // SmartDashboard.putNumber("Front ultrasonic bumper",
    // // Hardware.frontUltraSonic
    // // .getOffsetDistanceFromNearestBumper());
    //
    // // SmartDashboard.putNumber("Rear Ultrasonic",
    // // Hardware.rearUltraSonic.getDistanceFromNearestBumper());
    // // SmartDashboard.putNumber("Rear Ultrasonic Raw",
    // // Hardware.rearUltraSonic.getRefinedDistanceValue());
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

    // SmartDashboard.putNumber("Camera Center",
    // Hardware.driveWithCamera.getCameraCenterValue());
    // SmartDashboard.putNumber("Camera Center",
    // Hardware.driveWithCamera.getCameraCenterValue());

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
    SmartDashboard.putNumber("Gear",
            Hardware.transmission.getCurrentGear() + 1);

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

// in milliseconds
private double timeBetweenPrints = 10000;

// initial state
private double lastTimePrinted = 0.0;




}
