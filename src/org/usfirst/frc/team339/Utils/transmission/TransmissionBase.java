package org.usfirst.frc.team339.Utils.transmission;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Contains necessary functions that must be included in
 * each transmission type class created.
 * 
 * @author Ryan McGee
 * @written 7/17/2017
 */
public abstract class TransmissionBase
{
	// ================CONSTANTS================
	public static final double DEFAULT_JOYSTICK_DEADBAND = .2;
	// =========================================

	private final SpeedController[] motors;

	protected double[] gearRatios =
	{ .6, .8, 1 };

	// Will default to the highest gear available
	protected int currentGear = 0;

	private double inputJoystickDeadband = DEFAULT_JOYSTICK_DEADBAND;

	// The motors will start turning only once the joystick is past this
	// deadband.
	protected double currentJoystickDeadband = inputJoystickDeadband;

	/**
	 * Creates the TransmissionBase object. Should only be called by any subclasses,
	 * hence the abstract label.
	 * 
	 * @param leftRear
	 * @param rightRear
	 * @param leftFront
	 * @param rightFront
	 */
	public TransmissionBase(SpeedController leftRear, SpeedController rightRear, SpeedController leftFront,
			SpeedController rightFront)
	{
		this.motors = new SpeedController[4];
		this.motors[0] = leftRear;
		this.motors[1] = rightRear;
		this.motors[2] = leftFront;
		this.motors[3] = rightFront;
	}

	/**
	 * Creates the TransmissionBase object with a 2 wheel drive system. Should only
	 * be created by sub classes, hence the abstract label.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 */
	public TransmissionBase(SpeedController leftMotor, SpeedController rightMotor)
	{
		this.motors = new SpeedController[2];
		this.motors[0] = leftMotor;
		this.motors[1] = rightMotor;
	}

	/**
	 * The current types of transmissions available.
	 * 
	 * @author Ryan McGee
	 *
	 */
	public enum TransmissionType
	{
		/**
		 * Tank-style drive system with 2 driven traction wheels
		 * and two omniwheels for smooth steering.
		 */
		TRACTION,
		/**
		 * Tank-style drive system with four omniwheels each driven
		 * by a separate motor.
		 */
		TANK,
		/**
		 * Four driven mecanum wheels that allows strafing as well as
		 * linear movement and rotation.
		 */
		MECANUM
	}

	protected TransmissionType type = null;

	/**
	 * @return The type of transmission of a class extending TransmissionBase.
	 */
	public TransmissionType getType()
	{
		return type;
	}

	/**
	 * Describes which corner a motor is in when identifying it.
	 * 
	 * @author Ryan McGee
	 */
	public enum MotorPosition
	{
		LEFT, RIGHT, LEFT_FRONT, LEFT_REAR, RIGHT_FRONT, RIGHT_REAR, ALL
	}

	/**
	 * @param position
	 *            which corner the motor is in
	 * @return the motor controller object
	 */
	public SpeedController getSpeedController(MotorPosition position)
	{
		switch (position)
		{
		// Left is the same as left rear
		case LEFT:
		case LEFT_REAR:
			return this.motors[0];
		// Right is the same as right rear
		case RIGHT:
		case RIGHT_REAR:
			return this.motors[1];
		case LEFT_FRONT:
			return this.motors[2];
		case RIGHT_FRONT:
			return this.motors[3];
		default:
			return null;
		}

	}

	/**
	 * @return all speed controllers attached to this transmission, as an array.
	 */
	public SpeedController[] getAllSpeedControllers()
	{
		return this.motors;
	}

	/**
	 * Gets the robot ready for autonomous period.
	 */
	public void setForAutonomous()
	{
		disableDeadband();
		setMaxGearPercentage(1.0);
		setMaxGear();
	}

	/**
	 * Gets the robot ready for teleop period
	 * 
	 * @param maxGearPercentage
	 *            Speed percentage for top gear
	 */
	public void setForTeleop(double maxGearPercentage)
	{
		enableDeadband();
		setMaxGearPercentage(maxGearPercentage);
	}

	/**
	 * TODO Test gear system
	 * Sets the current gear for the robot. This will change the maximum
	 * speed of the robot for precise aiming/driving.
	 * 
	 * @param gear
	 *            The requested gear number. If outside the range, it will do
	 *            nothing.
	 */
	public void setGear(int gear)
	{
		if (gear >= 0 && gear < gearRatios.length)
			this.currentGear = gear;
	}

	/**
	 * Sets the percent multiplied by Transmission.
	 * 
	 * @param gear
	 *            Which gear should be changed: 0 is lowest, increasing.
	 * @param value
	 *            Percent decimal form: between 0 and 1.0
	 */
	public void setGearPercentage(int gear, double value)
	{
		if (value < 1 && value > 0 && gear < gearRatios.length && gear >= 0)
		{
			gearRatios[gear] = value;
		}
	}

	/**
	 * Sets the robot to the maximum gear available
	 * 
	 */
	public void setMaxGear()
	{
		this.currentGear = gearRatios.length - 1;
	}

	/**
	 * Sets the maximum gear to the value input.
	 * 
	 * @param value
	 *            Percent (0.0 to 1.0)
	 */
	public void setMaxGearPercentage(double value)
	{
		this.gearRatios[gearRatios.length - 1] = value;
	}

	/**
	 * Sets every gear ratio. Make sure that the lowest gear starts at 0, and the
	 * highest gear is at the max, to make sure the up-shifting and down-shifting
	 * works properly.
	 * 
	 * @param ratios
	 *            Percent multiplied by the transmission.drive functions
	 */
	public void setAllGearRatios(double... ratios)
	{
		this.gearRatios = ratios;
	}

	/**
	 * Adds one to the current gear of the robot, allowing the user to drive faster.
	 */
	public void upShift()
	{
		if (currentGear < gearRatios.length - 1)
			currentGear++;
	}

	/**
	 * Removes one from the current gear of the robot, allowing the user to drive
	 * slower.
	 */
	public void downShift()
	{
		if (currentGear > 0)
			currentGear--;
	}

	/**
	 * Shift gears using a up-shift and down-shift button.
	 * Also makes sure that holding the button will not trigger multiple shifts.
	 * 
	 * @param upShiftButton
	 *            The button that should change to the next higher gear
	 * @param downShiftButton
	 *            The button that should change to the next lowest gear
	 */
	public void shiftGears(boolean upShiftButton, boolean downShiftButton)
	{

		if (downShiftButton && !downShiftButtonStatus)
		{
			downShift();
		} else if (upShiftButton && !upShiftButtonStatus)
		{
			upShift();
		}

		upShiftButtonStatus = upShiftButton;
		downShiftButtonStatus = downShiftButton;
	}

	private boolean upShiftButtonStatus = false;

	private boolean downShiftButtonStatus = false;

	/**
	 * @return The gear number that is active
	 */
	public int getCurrentGear()
	{
		return this.currentGear;
	}

	/**
	 * @return
	 *         The percentage corresponding to the current gear
	 */
	public double getCurrentGearRatio()
	{
		return gearRatios[currentGear];
	}

	/**
	 * TODO test deadbands
	 * Sets the minimum value the joysticks must output in order for the robot to
	 * start moving.
	 * 
	 * @param deadband
	 *            Percentage value, ranging from 0.0 to 1.0, in decimals.
	 */
	public void setJoystickDeadband(double deadband)
	{
		this.inputJoystickDeadband = deadband;
		this.enableDeadband();
	}

	/**
	 * Turns off the deadband for use in auto.
	 */
	public void disableDeadband()
	{
		currentJoystickDeadband = 0;
	}

	/**
	 * Turns on the deadband for use in teleop.
	 */
	public void enableDeadband()
	{
		currentJoystickDeadband = inputJoystickDeadband;
	}

	/**
	 * Uses the formula for mapping one set of values to the other:
	 * y = mx + b
	 * 
	 * m = 1 / (1 - deadband)
	 * b = deadband * -m
	 * x = joystick input
	 * y = motor output
	 * 
	 * Therefore,
	 * motor output = (1 / (1 - deadband)) * joystick input
	 * + (1 - (1 / (1 - deadband)))
	 * 
	 * If this equation does not make much sense, try graphing it first
	 * as the original x = y, and then the scaled output starting at the
	 * deadband, and use the slope formula.
	 * 
	 * @param input
	 * @return The scaled value, if between -1 and -deadband or deadband and 1, or 0
	 *         if between -deadband and deadband.
	 */
	public double scaleJoystickForDeadband(double input)
	{
		double deadbandSlope = 1.0 / (1.0 - currentJoystickDeadband);
		double constant = -this.currentJoystickDeadband * deadbandSlope;

		if (input > this.currentJoystickDeadband)
			return (deadbandSlope * input) + constant;
		else if (input < -this.currentJoystickDeadband)
			return -((-deadbandSlope * input) + constant);

		// Set to 0 if it is between the deadbands.
		return 0.0;
	}

	/**
	 * Tells the robot to cut all power to the motors.
	 */
	public void stop()
	{
		for (SpeedController sc : motors)
			sc.set(0);
	}

}
