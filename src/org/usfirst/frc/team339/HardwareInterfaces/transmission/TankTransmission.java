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

	private final SpeedController leftFrontMotor;

	private final SpeedController rightFrontMotor;

	private final SpeedController leftRearMotor;

	private final SpeedController rightRearMotor;

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
	public TankTransmission(SpeedController leftFrontMotor, SpeedController rightFrontMotor,
			SpeedController leftRearMotor, SpeedController rightRearMotor)
	{
		this.leftFrontMotor = leftFrontMotor;
		this.rightFrontMotor = rightFrontMotor;
		this.leftRearMotor = leftRearMotor;
		this.rightRearMotor = rightRearMotor;

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
		this.drive(leftJoystick.getY(), rightJoystick.getY());
	}

	/**
	 * Controls the robot with the aid of deadbands and software gear ratios.
	 * 
	 * @param leftVal
	 * 			A decimal percentage, from -1.0 to 1.0
	 * @param rightVal
	 * 			A decimal percentage, from -1.0 to 1.0
	 */
	public void drive(double leftVal, double rightVal)
	{
		double leftY = super.scaleJoystickForDeadband(-leftVal) * super.gearRatios[super.currentGear];
		double rightY = super.scaleJoystickForDeadband(-rightVal) * super.gearRatios[super.currentGear];

		this.driveRaw(leftY, rightY);
	}

	@Override
	public void driveRaw(double leftVal, double rightVal)
	{
		leftFrontMotor.set(leftVal);
		leftRearMotor.set(leftVal);

		rightFrontMotor.set(rightVal);
		rightRearMotor.set(rightVal);
	}

	@Override
	public void stop()
	{
		this.leftFrontMotor.set(0.0);
		this.rightFrontMotor.set(0.0);
		this.leftRearMotor.set(0.0);
		this.rightRearMotor.set(0.0);
	}

	@Override
	public SpeedController getSpeedController(MotorPosition position)
	{
		switch (position)
		{
		case LEFT_FRONT:
			return this.leftFrontMotor;
		case LEFT_REAR:
			return this.leftRearMotor;
		case RIGHT_FRONT:
			return this.rightFrontMotor;
		case RIGHT_REAR:
			return this.rightRearMotor;
		default:
			return null;
		}
	}
}
