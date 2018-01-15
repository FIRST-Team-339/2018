package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * The most basic of drive systems: contains 2 motor controllers,
 * one on the left side of the robot, and one on the right.
 * 
 * @author Ryan McGee
 * @written 7/17/17
 */
public class TractionTransmission extends TransmissionBase
{
private final PWMSpeedController leftMotor;

private final PWMSpeedController rightMotor;

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
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

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
    this.drive(leftJoystick.getY(), rightJoystick.getY());
}


/**
 * Controls the robot with the help of gear ratios and deadbands.
 * 
 * @param leftVal
 *            The value sent to the left side, in decimal percentage (-1.0 to
 *            1.0)
 * @param rightVal
 *            The value sent to the right side, in decimal percentage (-1.0 to
 *            1.0)
 */
public void drive (double leftVal, double rightVal)
{
    double leftY = super.scaleJoystickForDeadband(-leftVal)
            * super.gearRatios[super.currentGear];
    double rightY = super.scaleJoystickForDeadband(-rightVal)
            * super.gearRatios[super.currentGear];

    this.driveRaw(leftY, rightY);
}

@Override
public void driveRaw (double leftVal, double rightVal)
{
    leftMotor.set(leftVal);
    rightMotor.set(rightVal);
}

@Override
public void stop ()
{
    this.leftMotor.set(0.0);
    this.rightMotor.set(0.0);
}

@Override
public SpeedController getSpeedController (MotorPosition position)
{
    switch (position)
        {
        case LEFT_FRONT:
            return this.leftMotor;
        case LEFT_REAR:
            return this.leftMotor;
        case RIGHT_FRONT:
            return this.rightMotor;
        case RIGHT_REAR:
            return this.rightMotor;
        default:
            return null;
        }
}

}
