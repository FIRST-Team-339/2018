package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Like the traction drive system, except with four motors, usually all as
 * omni-wheels.
 * Or, tank drive with paired motors. Each joystick controls one side of the
 * robot.
 * 
 * @author Ryan McGee
 * @written 7/21/17
 */
public class TankTransmission extends TransmissionBase
{
	/**
	 * Creates the Transmission object.
	 * 
	 * @param leftFrontMotor
	 *            The left-front motor controller
	 * @param rightFrontMotor
	 *            The right-front motor controller
	 * @param leftRearMotor
	 *            The left-rear motor controller
	 * @param rightRearMotor
	 *            The right-rear motor controller
	 */
	public TankTransmission(SpeedController leftRearMotor, SpeedController rightRearMotor,
			SpeedController leftFrontMotor, SpeedController rightFrontMotor)
	{
		super(leftRearMotor, rightRearMotor, leftFrontMotor, rightFrontMotor);

		super.type = TransmissionType.TANK;
	}

	/**
	 * Creates the Transmission object with two speed controllers.
	 * (either through a Y cable, or only 2 driven wheels)
	 * 
	 * @param leftMotor
	 * 			The motor controlling the left side of the robot
	 * @param rightMotor
	 * 			The motor controlling the right side of the robot
	 */
	public TankTransmission(SpeedController leftMotor, SpeedController rightMotor)
	{
		super(leftMotor, rightMotor);
	}

	/**
	 * Controls the robot with the aid of deadbands and software gear ratios.
	 * 
	 * @param leftJoystick
	 *            The joystick that will control the left side of the robot
	 * @param rightJoystick
	 *            The joystick that will control the right side of the robot
	 */
	public void drive(Joystick leftJoystick, Joystick rightJoystick)
	{
		this.drive(-leftJoystick.getY(), -rightJoystick.getY());
	}

	/**
	 * Drives the transmission based on a Tank drive system,
	 *  where left controls the left wheels, and right controls the right wheels. 
	 *  
	 *  Uses joystick deadbands and gear ratios.
	 *  
	 * @param leftVal
	 *            Percentage, (-1.0 to 1.0)
	 * @param rightVal
	 *            Percentage, (-1.0 to 1.0)
	 */
	public void drive(double leftVal, double rightVal)
	{
		double leftOut = super.scaleJoystickForDeadband(leftVal) * super.getCurrentGearRatio();
		double rightOut = super.scaleJoystickForDeadband(rightVal) * super.getCurrentGearRatio();

		super.driveRaw(leftOut, rightOut);
	}
}
