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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import org.usfirst.frc.team339.HardwareInterfaces.SingleThrowSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.SixPositionSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;


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
public static TalonSRX rightMotor = new TalonSRX(1);

public static TalonSRX leftMotor = new TalonSRX(2);

public static TalonSRX liftingMotor = new TalonSRX(3);

public static TalonSRX cubeIntakeMotor = new TalonSRX(4);

// ------------------------------------
// Victor Classes
// ------------------------------------

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

public static SingleThrowSwitch disableAutoSwitch = new SingleThrowSwitch(
        23);

public static SixPositionSwitch autoStateSwitch = new SixPositionSwitch(
        1, 2, 3, 4, 5, 6);

// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------

public static Encoder leftRearEncoder = new Encoder(10, 11);

public static Encoder rightRearEncoder = new Encoder(12, 13);

public static Encoder leftFrontEncoder = new Encoder(14, 15);

public static Encoder rightFrontEncoder = new Encoder(16, 17);

public static Encoder liftingEncoder = new Encoder(18, 19);

public static Encoder cubeIntakeEncoder = new Encoder(20, 21);

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

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------

public static UltraSonic frontUltraSonic = new UltraSonic(1);

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

// ------------------------------------
// Drive system
// ------------------------------------

// =====================================================================
// Drive classes
// =====================================================================

// -------------------
// Assembly classes (e.g. forklift)
// -------------------

// ------------------------------------
// Utility classes
// ------------------------------------

} // end class
