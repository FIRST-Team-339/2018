package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * One of the more complex drive systems: uses 4 mecanum wheels to allow
 * for lateral movement as well as normal tank controls.
 * 
 * @author Ryan McGee
 * @written 7/21/17
 */
public class MecanumTransmission extends TransmissionBase
{

private double strafeCushion = .2;

private double directionalDeadband = Math.toRadians(0);// Disabled by default.

/**
 * Creates the MecanumTransmission object.
 * 
 * @param leftFrontMotor
 *            The left front motor controller
 * @param rightFrontMotor
 *            The right front motor controller
 * @param leftRearMotor
 *            The left rear motor controller
 * @param rightRearMotor
 *            The right rear motor controller
 */
public MecanumTransmission (SpeedController leftFrontMotor,
        SpeedController rightFrontMotor,
        SpeedController leftRearMotor, SpeedController rightRearMotor)
{
    super(leftRearMotor, rightRearMotor, leftFrontMotor,
            rightFrontMotor);

    super.type = TransmissionType.MECANUM;
}

/**
 * Drives the robot with the aid of a joystick deadband, directional deadbands,
 * and software gear ratios.
 * 
 * @param joystick
 *            A 3-axis joystick to control 2 axis movement plus turning.
 */
public void drive (Joystick joystick)
{
    if (joystick.getTrigger() == true)
        this.drive(joystick.getMagnitude(),
                joystick.getDirectionRadians(),
                joystick.getZ());
    else
        this.drive(joystick.getMagnitude(),
                joystick.getDirectionRadians(),
                0);
}

/**
 * Drives the robot with mecanum with raw values, taking into account
 * joystick deadbands and gear ratios.
 * 
 * 
 * @param magnitude
 *            The magnitude of the joystick, (0.0 to 1.0)
 * @param direction
 *            The direction of the joystick in radians (-PI to PI)
 * @param rotation
 *            The rotation of the joystick, (-1.0 to 1.0)
 */
public void drive (double magnitude, double direction, double rotation)
{
    double altMagnitude, altDirection, altRotation;

    altMagnitude = super.scaleJoystickForDeadband(magnitude)
            * super.gearRatios[super.currentGear];
    altDirection = direction;
    altRotation = super.scaleJoystickForDeadband(rotation)
            * super.gearRatios[super.currentGear];

    // Check between the deadbands for the strafing cushion and and 90
    // degree "snap".
    if (direction > (-Math.PI / 2.0) - (directionalDeadband / 2.0)
            && direction < (-Math.PI / 2.0)
                    + (directionalDeadband / 2.0))
        {
        altDirection = -Math.PI / 2.0;
        altMagnitude += this.strafeCushion;
        }
    else if (direction > (Math.PI / 2.0) - (directionalDeadband / 2.0)
            && direction < (Math.PI / 2.0)
                    + (directionalDeadband / 2.0))
        {
        altDirection = Math.PI / 2.0;
        altMagnitude += this.strafeCushion;
        }

    double leftFrontVal = Math.cos(direction - (Math.PI / 4.0));
    double rightFrontVal = Math.cos(direction + (Math.PI / 4.0));
    double rightRearVal = leftFrontVal;// The corner's equations are
    double leftRearVal = rightFrontVal;// equal to each other.

    super.getSpeedController(MotorPosition.LEFT_FRONT)
            .set(inRange((magnitude * leftFrontVal) + rotation));
    super.getSpeedController(MotorPosition.LEFT_REAR)
            .set(inRange((magnitude * leftRearVal) + rotation));
    super.getSpeedController(MotorPosition.RIGHT_FRONT)
            .set(inRange((magnitude * rightFrontVal) - rotation));
    super.getSpeedController(MotorPosition.RIGHT_REAR)
            .set(inRange((magnitude * rightRearVal) - rotation));

}

/**
 * Sets the angle deadband where the direction snaps to a 90 degree angle.
 * 
 * @param value
 *            deadband, in degrees
 */
public void setDirectionalDeadband (double value)
{
    this.directionalDeadband = Math.toRadians(value);
}

/**
 * Sets the percentage added to the magnitude while strafing.
 * This is only used inside the directional deadband, so if it is set to 0,
 * then it will not take affect.
 * 
 * @param value
 *            The value that strafeCusion will be set to.
 */
public void setStrafeCusion (double value)
{
    this.strafeCushion = value;
}

/**
 * Checks if the value input is in between -1 and 1 to keep it in range for
 * motor inputs.
 * 
 * @param val
 *            The input value
 * @return The correctly ranged value
 */
private double inRange (double val)
{
    if (val > 1)
        return 1;
    else if (val < -1)
        return -1;

    return val;
}

}
