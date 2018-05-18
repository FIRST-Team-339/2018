package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

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
	 * @param leftSide the grouped motor controllers on the left side of the robot
	 * 
	 * @param rightSide the grouped motor controllers on the right side of the robot
	 */
	public TankTransmission(SpeedControllerGroup leftSide, SpeedControllerGroup rightSide)
	{
		super(leftSide, rightSide);

		super.type = TransmissionType.TANK;
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
