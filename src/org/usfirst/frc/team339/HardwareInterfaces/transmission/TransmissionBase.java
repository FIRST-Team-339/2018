package org.usfirst.frc.team339.HardwareInterfaces.transmission;

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
protected double[] gearRatios =
    {.6, .8, 1};

// Will default to the highest gear available
protected int currentGear = 0;

// The motors will start turning only once the joystick is past this
// deadband.
protected double joystickDeadband = .2;

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
public TransmissionType getType ()
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
LEFT_FRONT, LEFT_REAR, RIGHT_FRONT, RIGHT_REAR, ALL
    }

/**
 * @param position
 *            which corner the motor is in
 * @return the motor controller object
 */
public abstract SpeedController
        getSpeedController (MotorPosition position);

/**
 * TODO Test gear system
 * Sets the current gear for the robot. This will change the maximum
 * speed of the robot for precise aiming/driving.
 * 
 * @param gear
 *            The requested gear number. If outside the range, it will do
 *            nothing.
 */
public void setGear (int gear)
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
public void setGearPercentage (int gear, double value)
{
    if (value < 1 && value > 0 && gear < gearRatios.length && gear >= 0)
        {
        gearRatios[gear] = value;
        }
}

/**
 * Sets the maximum number of speeds the robot can shift to
 * 
 * @param numberOfGears
 *            Number of gears to set
 */
public void setMaxGears (int numberOfGears)
{
    this.gearRatios = new double[numberOfGears];
}

/**
 * Sets every gear ratio. Make sure that the lowest gear starts at 0, and the
 * highest gear is at the max, to make sure the up-shifting and down-shifting
 * works properly.
 * 
 * @param ratios
 *            Percent multiplied by the transmission.drive functions
 */
public void setAllGearRatios (double... ratios)
{
    this.gearRatios = ratios;
}

/**
 * Adds one to the current gear of the robot, allowing the user to drive faster.
 */
public void upShift ()
{
    if (currentGear < gearRatios.length - 1)
        currentGear++;
}

/**
 * Removes one from the current gear of the robot, allowing the user to drive
 * slower.
 */
public void downShift ()
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
public void shiftGears (boolean upShiftButton, boolean downShiftButton)
{

    if (downShiftButton && !downShiftButtonStatus)
        {
        downShift();
        }
    else if (upShiftButton && !upShiftButtonStatus)
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
public int getCurrentGear ()
{
    return this.currentGear;
}

/**
 * @return
 *         The percentage corresponding to the current gear
 */
public double getCurrentGearRatio ()
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
public void setJoystickDeadband (double deadband)
{
    this.joystickDeadband = deadband;
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
public double scaleJoystickForDeadband (double input)
{
    double deadbandSlope = 1.0 / (1.0 - joystickDeadband);
    double constant = -this.joystickDeadband * deadbandSlope;

    if (input > this.joystickDeadband)
        return (deadbandSlope * input) + constant;
    else if (input < -this.joystickDeadband)
        return -((-deadbandSlope * input) + constant);

    // Set to 0 if it is between the deadbands.
    return 0.0;
}

/**
 * Tells the robot to cut all power to the motors.
 */
public abstract void stop ();

/**
 * Drives either the left side or the right side of the robot,
 * without the use of deadbands and gear ratios.
 * Useful in autonomous driving situations
 * 
 * Also forces tank style driving if there are other forms.
 * 
 * @param leftVal
 *            Percentage sent to the left-side motors (-1.0 to 1.0)
 * @param rightVal
 *            Percentage sent to the right-side motors (-1.0 to 1.0)
 */
public abstract void driveRaw (double leftVal, double rightVal);

}
