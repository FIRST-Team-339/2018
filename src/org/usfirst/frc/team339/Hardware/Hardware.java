// ====================================================================
// FILE NAME: Hardware.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 2, 2011
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file contains all of the global definitions for the
// hardware objects in the system
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.Hardware;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc.team339.HardwareInterfaces.DoubleThrowSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.DriveWithCamera;
import org.usfirst.frc.team339.HardwareInterfaces.KilroyEncoder;
import org.usfirst.frc.team339.HardwareInterfaces.KilroySPIGyro;
import org.usfirst.frc.team339.HardwareInterfaces.LVMaxSonarEZ;
import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import org.usfirst.frc.team339.HardwareInterfaces.MomentarySwitch;
import org.usfirst.frc.team339.HardwareInterfaces.RobotPotentiometer;
import org.usfirst.frc.team339.HardwareInterfaces.SingleThrowSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.SixPositionSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TankTransmission;
import org.usfirst.frc.team339.Utils.CubeManipulator;
import org.usfirst.frc.team339.Utils.ScaleAlignment;
import org.usfirst.frc.team339.Utils.drive.Drive;
import org.usfirst.frc.team339.vision.VisionProcessor;
import org.usfirst.frc.team339.vision.VisionProcessor.CameraModel;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * -------------------------------------------------------
 * puts all of the hardware declarations into one place. In addition, it makes
 * them available to both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */

public class Hardware
{
// ------------------------------------
// Public Constants
// ------------------------------------

// -------------------------------------
// Private Constants
// -------------------------------------

// ---------------------------------------
// Hardware Tunables
// ---------------------------------------
public static boolean onNessie = false;

public static boolean on2018 = true;

// **********************************************************
// DIGITAL I/O CLASSES
// **********************************************************

// ====================================
// PWM classes
// ====================================

// ------------------------------------
// Jaguar classes
// ------------------------------------

// ------------------------------------
// Talon classes
// ------------------------------------
public static Talon rightDriveMotor = new Talon(2);

public static Talon leftDriveMotor = new Talon(3);

// ------------------------------------
// Victor Classes
// ------------------------------------
public static VictorSP liftingMotor = new VictorSP(0);

public static VictorSP cubeIntakeMotor = new VictorSP(1);

public static VictorSP intakeDeployArm = new VictorSP(4);

// ------------------------------------
// Servo classes
// ------------------------------------
public static Servo climbingMechanismServo = new Servo(5);
// Documentation says 200* servo

public static Servo intakeArmPositionServo = new Servo(6);
// shotwell says 180* servo

// ====================================
// CAN classes
// ====================================

public static WPI_TalonSRX rightCANMotor = new WPI_TalonSRX(14);

public static WPI_TalonSRX leftCANMotor = new WPI_TalonSRX(11);

public static WPI_TalonSRX rightRearCANMotor = new WPI_TalonSRX(12);

public static WPI_TalonSRX leftRearCANMotor = new WPI_TalonSRX(13);

// ====================================
// Relay classes
// ====================================
public static Relay ringLightRelay = new Relay(1);

// JANKY temporary fix until wpi gets their crap together with the Relay
// class.
public static DigitalOutput tempRelay = new DigitalOutput(0);

// ====================================
// Digital Inputs
// ====================================
// ------------------------------------
// Single and double throw switches
// ------------------------------------
public static SingleThrowSwitch leftAutoSwitch = new SingleThrowSwitch(
        20);

public static SingleThrowSwitch rightAutoSwitch = new SingleThrowSwitch(
        25);

public static DoubleThrowSwitch disableAutonomousSwitch = new DoubleThrowSwitch(
        leftAutoSwitch, rightAutoSwitch);

public static SingleThrowSwitch demoModeSwitch = new SingleThrowSwitch(
        8);

public static SixPositionSwitch autoSixPosSwitch = new SixPositionSwitch(
        1, 2, 3, 4, 5, 6);

// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------
public static KilroyEncoder leftRearDriveEncoder = new KilroyEncoder(10,
        11);

public static KilroyEncoder rightRearDriveEncoder = new KilroyEncoder(
        12, 13);

public static KilroyEncoder leftFrontDriveEncoder = new KilroyEncoder(
        14, 15);

public static KilroyEncoder rightFrontDriveEncoder = new KilroyEncoder(
        16, 17);

public static Encoder liftingEncoder = new Encoder(18, 19);

public static Encoder intakeDeployEncoder = new Encoder(23, 24);

// -----------------------
// Wiring diagram
// -----------------------
// Orange - Red PWM 1
// Yellow - White PWM 1 Signal
// Brown - Black PWM 1 (or PWM 2)
// Blue - White PWM 2 Signal
// For the AMT103 Encoders UNVERIFIED
// B - White PWM 2
// 5V - Red PWM 1 or 2
// A - White PWM 1
// X - index channel, unused
// G - Black PWM 1 or 2
// see http://www.cui.com/product/resource/amt10-v.pdf page 4 for Resolution
// (DIP Switch) Settings (currently all are off)

// -------------------------------------
// Red Light/IR Sensor class
// -------------------------------------
public static LightSensor armIR = new LightSensor(21);// TODO check port for
                                                      // 2018 robot

public static LightSensor redLight = new LightSensor(7);

// public static LightSensor leftRedLight = new LightSensor(8);

public static LightSensor cubePhotoSwitch = new LightSensor(22);

// ====================================
// I2C Classes
// ====================================

// **********************************************************
// SOLENOID I/O CLASSES
// **********************************************************
// ====================================
// Compressor class - runs the compressor
// ====================================

// ====================================
// Pneumatic Control Module
// ====================================

// ====================================
// Solenoids
// ====================================
// ------------------------------------
// Double Solenoids
// ------------------------------------

// ------------------------------------
// Single Solenoids
// ------------------------------------

// **********************************************************
// ANALOG I/O CLASSES
// **********************************************************
// ====================================
// Analog classes
// ====================================
// ------------------------------------
// Gyro class
// ------------------------------------

// P/N ADW22307
public static AnalogGyro gyroAnalog = new AnalogGyro(0);

// -------------------------------------
// Potentiometers
// -------------------------------------
public static RobotPotentiometer delayPot = new RobotPotentiometer(2,
        300);

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------
public static LVMaxSonarEZ frontUltraSonic = new LVMaxSonarEZ(3);

public static LVMaxSonarEZ rearUltraSonic = new LVMaxSonarEZ(1);

// =====================================
// SPI Bus
// =====================================

// -------------------------------------
// Analog Interfaces
// -------------------------------------
public static KilroySPIGyro gyro = new KilroySPIGyro(true);

// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------

public static VisionProcessor axisCamera = new VisionProcessor(
        "10.3.39.11", CameraModel.AXIS_M1013, tempRelay);

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------
public static UsbCamera USBCam = CameraServer.getInstance()
        .startAutomaticCapture(0);

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------
// public static UsbCamera USBCamUp = CameraServer.getInstance()
// .startAutomaticCapture(1);

// **********************************************************
// DRIVER STATION CLASSES
// **********************************************************
// ------------------------------------
// DriverStations class
// ------------------------------------
public static DriverStation driverStation = DriverStation.getInstance();

// ------------------------------------
// Joystick classes
// ------------------------------------
public static Joystick leftDriver = new Joystick(0);

public static Joystick rightDriver = new Joystick(1);

public static Joystick leftOperator = new Joystick(2);

public static Joystick rightOperator = new Joystick(3);

// ------------------------------------
// Momentary Switches
// ------------------------------------
public static MomentarySwitch visionTestButton = new MomentarySwitch(
        leftOperator, 2,
        false /* starting state */);

public static MomentarySwitch climbButton = new MomentarySwitch(
        rightOperator, 10,
        false /* starting state */);

// **********************************************************
// Kilroy's Ancillary classes
// **********************************************************

// -------------------------------------
// PID tuneables
// -------------------------------------

// -------------------------------------
// PID classes
// -------------------------------------

// ------------------------------------
// Utility classes
// ------------------------------------
public static final Timer autoTimer = new Timer();

// ------------------------------------
// Transmission class
// ------------------------------------
public static TankTransmission transmission = new TankTransmission(
        new SpeedControllerGroup(leftCANMotor, leftRearCANMotor),
        new SpeedControllerGroup(rightCANMotor, rightRearCANMotor));
// public static MecanumTransmission transmission = new MecanumTransmission(
// leftCANMotor,
// rightCANMotor, leftRearCANMotor, rightRearCANMotor);

// public static MecanumTransmission transmission = new MecanumTransmission(
// leftRearCANMotor, rightRearCANMotor, leftCANMotor,
// rightCANMotor);

// ------------------------------------
// Drive system
// ------------------------------------
public static Drive drive = new Drive(transmission,
        leftRearDriveEncoder,
        rightRearDriveEncoder,
        leftFrontDriveEncoder, rightFrontDriveEncoder, gyro);
// TODO CHANGE TO FRONT ENCODERS ON REAL ROBOT

// TODO change back to this once relays actually work
public static DriveWithCamera driveWithCamera = new DriveWithCamera(
        transmission, leftFrontDriveEncoder,
        rightFrontDriveEncoder, frontUltraSonic, rearUltraSonic, gyro,
        axisCamera);

// this is a janky fix for the ringlight not working
// public static DriveWithCamera driveWithCamera = new DriveWithCamera(
// transmission, leftFrontDriveEncoder, rightFrontDriveEncoder,
// frontUltraSonic, rearUltraSonic, gyro, axisCamera, tempRelay);

// -------------------
// Assembly classes (e.g. forklift)
// -------------------
public static CubeManipulator cubeManipulator = new CubeManipulator(
        liftingMotor, cubeIntakeMotor, cubePhotoSwitch,
        liftingEncoder, intakeDeployArm, intakeDeployEncoder, autoTimer,
        intakeArmPositionServo, armIR);

public static ScaleAlignment scaleAlignment = new ScaleAlignment(
        rearUltraSonic);
} // end class
