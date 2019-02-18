package org.usfirst.frc.team339.Utils;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
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
// the first part of the path for logs
private final String LOG_BASIC_PATH = "/home/lvuser";

// the file path where we save the logs we take
private final String SAVE_LOG_PATH = LOG_BASIC_PATH + "/logs";

private final static String timeStamp = new SimpleDateFormat("MMddHHmm")
        .format(new Date());

private final int numLogFolders = 10;


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


            // =================================
            // Motor
            // Prints the value of motors
            // =================================
            // System.out.println(
            // "Right Drive Motor "
            // + Hardware.rightDriveMotor.get());
            //
            // System.out.println(
            // "Left Drive Motor "
            // + Hardware.leftDriveMotor.get());
            //


            // System.out.println(
            // "Right Drive Motor "
            // + Hardware.rightFrontCANMotor.get());

            // System.out.println(
            // "Left Drive Motor "
            // + Hardware.leftFrontCANMotor.get());


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

            // System.out.println(
            // "Demo swich isOn" + Hardware.demoModeSwitch.isOn());

            // ---------------------------------
            // Encoders
            // ---------------------------------
            // System.out.println("Left Front Encoder Inches = "
            // + Hardware.leftFrontDriveEncoder.getDistance());
            //
            //
            // System.out.println("Left Front Encoder Ticks "
            // + Hardware.leftFrontDriveEncoder.get());

            // System.out.println("RF encoder type"
            // + Hardware.rightFrontDriveEncoder.getSensorType());

            // System.out.println("Right Front Inches = "
            // + Hardware.rightFrontDriveEncoder.getDistance());
            //
            // System.out.println("Right Front Ticks "
            // + Hardware.rightFrontDriveEncoder.get());

            // System.out.println("Left Rear Encoder Inches = "
            // + Hardware.leftRearDriveEncoder.getDistance());
            //
            // System.out.println("Left Rear Encoder Ticks "
            // + Hardware.leftRearDriveEncoder.get());

            // System.out.println("RR encoder type"
            // + Hardware.rightRearDriveEncoder.getSensorType());
            // System.out.println("Right Rear Inches = "
            // + Hardware.rightRearDriveEncoder.getDistance());
            //
            // System.out.println("Right Rear Ticks "
            // + Hardware.rightRearDriveEncoder.get());


            // System.out.println("deploy Ticks "
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
    // clock determining change

    if ((System.currentTimeMillis()
            - lastTimePrinted) >= this.getTimeBetweenPrints())
        {
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
         SmartDashboard.putNumber("Left Front Encoder Inches",
         Hardware.leftFrontDriveEncoder.getDistance());
        //
        SmartDashboard.putNumber("Left Front Encoder Ticks",
                Hardware.leftFrontDriveEncoder.get());
        //
        //
         SmartDashboard.putNumber("Right Front Encoder Inches",
         Hardware.rightFrontDriveEncoder.getDistance());
        //
        //
        SmartDashboard.putNumber("Right Front Encoder Ticks",
                Hardware.rightFrontDriveEncoder.get());
        // //
         SmartDashboard.putNumber("Left Rear Encoder Inches",
         Hardware.leftRearDriveEncoder.getDistance());
        //
        //
        SmartDashboard.putNumber("Left Rear Encoder Ticks",
                Hardware.leftRearDriveEncoder.get());
        //
         SmartDashboard.putNumber("Right Rear Encoder Inches",
         Hardware.rightRearDriveEncoder.getDistance());
        // //
        SmartDashboard.putNumber("Right Rear Encoder Ticks",
                Hardware.rightRearDriveEncoder.get());

        // SmartDashboard.putNumber("Lift Encoder Ticks",
        // Hardware.liftingEncoder.get());
        //
        SmartDashboard.putNumber("Deploy Encoder Ticks",
                Hardware.intakeDeployEncoder.get());

        SmartDashboard.putNumber("Lift Encoder Ticks",
                Hardware.liftingEncoder.get());

        SmartDashboard.putNumber("Lift Encoder Inches",
                Hardware.liftingEncoder.getDistance());


        SmartDashboard.putString("max forklift height",
                Hardware.cubeManipulator.currentForkliftMaxHeight + "");

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
        // SmartDashboard.putNumber("Delay Pot",
        // Hardware.delayPot.get(0, 5));
        //
        // // ---------------------------------
        // // GYRO
        // // ---------------------------------
        //
        //
        // SmartDashboard.putNumber("AnalogGyro",
        // Hardware.gyroAnalog.getAngle());

        // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());


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

        // -------------------------------------
        // Analog Interfaces
        // -------------------------------------

        // SmartDashboard.putNumber("Gyro", Hardware.gyro.getAngle());

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



        SmartDashboard.updateValues();
        }
}


public void saveToRoboRIO ()
{
    String fileName = "";
    try
        {
        Runtime.getRuntime()
                .exec("mkdir -p " + SAVE_LOG_PATH);
        /*
         * if TS is blank
         * if >9 delete all but first 9
         * Create new file under logs directory named TS & write TS as a string
         * & write into TS}
         * else timestamp is filled, write into file TS}
         */
        // if TS fileName is blank
        if (fileName == "")
            {
            // remove all but top 9 files
            fileName = "log_" + timeStamp;
            // create new file
            // under
            // directory
            // SAVE_LOG_PATH+timeStamp


            }
        // system command that creates the path the image will be saved in
        /*
         * Run.Runtime()
         * .exec("ls -1rtd *log | head -n$(($(ls -1d *log | wc -l)-"
         * + numLogFolders + ")) | xargs rm");
         */
        System.out.println("delete delete delete delete");
        Runtime.getRuntime()
                .exec("sleep 300; ls -trd *.log | head -n$(expr $(ls -d *.log | wc -l) - 10) | xargs"
                        + "rm");


        /*
         * Runtime.getRuntime()
         * .exec("printf \"" + input + "\" >> " + fileName
         * + ".log");
         */
        } // end try
    catch (IOException e)
        {
        e.printStackTrace();
        } // catch

}

// in milliseconds
private double timeBetweenPrints = 2000;

// initial state
private double lastTimePrinted = 0.0;

}
