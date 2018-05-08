package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSpeedController;

/**
 * The most basic of drive systems: contains 2 motor controllers,
 * one on the left side of the robot, and one on the right.
 * 
 * @author Ryan McGee
 * @written 7/17/17
 */
public class TractionTransmission extends TransmissionBase
{

/**
 * Constructs the transmission object with 2 motors.
 * 
 * @param leftMotor
 *            The controller connected to the left motor
 * @param rightMotor
 *            The controller connected to the right motor
 */
public TractionTransmission (PWMSpeedController leftMotor,
        PWMSpeedController rightMotor)
{
    super(leftMotor, rightMotor);

    super.type = TransmissionType.TRACTION;
}

/**
 * Controls the robot with the help of gear ratios and deadbands.
 * 
 * @param leftJoystick
 *            The joystick that will control the left side of the robot
 * @param rightJoystick
 *            The joystick that will control the right side of the robot
 */
public void drive (Joystick leftJoystick, Joystick rightJoystick)
{
    this.drive(-leftJoystick.getY(), -rightJoystick.getY());
}

/**
 * Drives the robot based on raw double values, scaled for the deadband and gear
 * ratios.
 * 
 * @param leftVal
 * @param rightVal
 */
public void drive (double leftVal, double rightVal)
{
    double leftOut = super.scaleJoystickForDeadband(leftVal)
            * super.getCurrentGearRatio();
    double rightOut = super.scaleJoystickForDeadband(rightVal)
            * super.getCurrentGearRatio();

    super.driveRaw(leftOut, rightOut);
}

}
