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

import org.usfirst.frc.team339.HardwareInterfaces.DoubleThrowSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import org.usfirst.frc.team339.HardwareInterfaces.RobotPotentiometer;
import org.usfirst.frc.team339.HardwareInterfaces.SingleThrowSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.SixPositionSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.Drive;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TractionTransmission;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;


// -------------------------------------------------------
/**
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
public static Servo climbingMechanismServo = new Servo(5); // 5 is not set

// ------------------------------------
// Public Constants
// ------------------------------------

// -------------------------------------
// Private Constants
// -------------------------------------

// ---------------------------------------
// Hardware Tunables
// ---------------------------------------

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
public static Victor liftingMotor = new Victor(0);

public static Victor cubeIntakeMotor = new Victor(1);

public static Victor intakeDeployArm = new Victor(4);

// ====================================
// CAN classes
// ====================================

// ====================================
// Relay classes
// ====================================

public static Relay ringLightRelay = new Relay(0);

// ====================================
// Digital Inputs
// ====================================
// ------------------------------------
// Single and double throw switches
// ------------------------------------

public static DoubleThrowSwitch autoSelectorSwitch = new DoubleThrowSwitch(
        20, 21);

public static SixPositionSwitch autoStateSwitch = new SixPositionSwitch(
        1, 2, 3, 4, 5, 6);

// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------

public static Encoder leftRearDriveEncoder = new Encoder(10, 11);

public static Encoder rightRearDriveEncoder = new Encoder(12, 13);

public static Encoder leftFrontDriveEncoder = new Encoder(14, 15);

public static Encoder rightFrontDriveEncoder = new Encoder(16, 17);

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

public static LightSensor rightRedLight = new LightSensor(7);

public static LightSensor leftRedLight = new LightSensor(8);

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

// -------------------------------------
// Potentiometers
// -------------------------------------
// -------------------------------------

public static RobotPotentiometer delayPot = new RobotPotentiometer(2,
        270);

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------

public static UltraSonic frontUltraSonic = new UltraSonic(0);

public static UltraSonic rearUltraSonic = new UltraSonic(1);

// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves
// -------------------------------------

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
public static Joystick rightDriver = new Joystick(0);

public static Joystick leftDriver = new Joystick(1);

public static Joystick rightOperator = new Joystick(2);

public static Joystick leftOperator = new Joystick(3);

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
// Transmission class
// ------------------------------------
public static TractionTransmission tractionDrive = new TractionTransmission(
        leftDriveMotor, rightDriveMotor);

// ------------------------------------
// Drive system
// ------------------------------------
public static Drive autoDrive = new Drive(tractionDrive,
        leftFrontDriveEncoder, rightFrontDriveEncoder, frontUltraSonic,
        rearUltraSonic, null);
// -------------------
// Assembly classes (e.g. forklift)
// -------------------

// ------------------------------------
// Utility classes
// ------------------------------------

public static final Timer autoTimer = new Timer();

} // end class
