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
public TankTransmission (SpeedController leftFrontMotor,
        SpeedController rightFrontMotor,
        SpeedController leftRearMotor, SpeedController rightRearMotor)
{
    super(leftRearMotor, rightRearMotor, leftFrontMotor,
            rightFrontMotor);

    super.type = TransmissionType.TANK;
}

/**
 * 
 * @param leftMotor
 * @param rightMotor
 */
public TankTransmission (SpeedController leftMotor,
        SpeedController rightMotor)
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
public void drive (Joystick leftJoystick, Joystick rightJoystick)
{
    super.drive(-leftJoystick.getY(), -rightJoystick.getY());
}

}
