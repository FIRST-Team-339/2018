package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;

/**
 * Class for forklift and intake subsystems
 * 
 * @author Becky Button
 *
 */
public class Forklift
{
private Victor forkliftMotor = null;

private Victor intakeMotor = null;

private Victor intakeDeployMotor = null;

private Encoder intakeDeployEncoder = null;

private LightSensor intakeSwitch = null;

private Encoder forkliftEncoder = null;

/**
 * @param forkliftMotor
 * @param intakeMotor
 * @param intakeSwitch
 * @param forkliftEncoder
 * @param intakeDeploy
 */
public Forklift (Victor forkliftMotor, Victor intakeMotor,
        LightSensor intakeSwitch, Encoder forkliftEncoder,
        Victor intakeDeploy, Encoder intakeDeployEncoder)
{
    this.forkliftMotor = forkliftMotor;
    this.intakeMotor = intakeMotor;
    this.intakeSwitch = intakeSwitch;
    this.forkliftEncoder = forkliftEncoder;
    this.intakeDeployMotor = intakeDeploy;
    this.intakeDeployEncoder = intakeDeployEncoder;

}

/**
 * gets the distance value from the encoder mounted to the forklift
 * YO NEWBIES -- we want to slow down when the forklift gets above a certain
 * height, you'll need to use this method to determine height
 * 
 * @return the height of the forklift in inches
 */
public double getForkliftHeight ()
{
    return this.forkliftEncoder.getDistance();
}

/**
 * YO NEWBIES-- USE THIS TO MOVE THE FORKLIFT
 * 
 * @param operatorJoystick
 *            Joystick object
 */
public void moveForkliftWithController (Joystick operatorJoystick)
{
    if (getForkliftHeight() < FORKLIFT_MAX_HEIGHT
            || getForkliftHeight() > FORKLIFT_MIN_HEIGHT)
        {
        this.forkliftMotor.set(
                (operatorJoystick.getY()) * FORKLIFT_SPEED_COEFFICIENT);
        }
    else
        {
        this.forkliftMotor.set(0);
        }
}

/**
 * Intake runs until the light switch is on
 * 
 * NEWBIES USE ON A BUTTON!!!!
 * 
 * @return true if the the cube is in, the light switch is off
 */
public boolean intakeCube ()
{
    if (this.intakeSwitch.isOn() == false)
        {
        this.intakeMotor.set(INTAKE_SPEED);
        return false;
        }
    this.intakeMotor.set(0);
    return true;
}

/**
 * Runs intake in reverse of intakeCube()
 * 
 * NEWBIES USE ON A BUTTON!!!!
 * 
 * @return true if the light sensor equals true
 */
public boolean pushOutCube ()
{
    if (this.intakeSwitch.isOn() == true)
        {
        this.intakeMotor.set(-this.INTAKE_SPEED);
        return false;
        }
    this.intakeMotor.set(0);
    return true;
}

/**
 * Method for moving deploy arm to the down position
 * 
 * NEWBIES CAN IGNORE -- THIS ONLY APPLIES TO AUTO
 * 
 * @return true if the arm is down, false if it is still moving
 */
public boolean deployCubeIntake ()
{
    if (this.intakeDeployEncoder.getDistance() <= INTAKE_ANGLE)
        {
        this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
        }
    else
        {
        this.intakeDeployMotor.set(0);
        }
    return true;
}

/**
 * HEY NEWBIES -- USE THIS FOR THE THE INTAKE OVERRIDE
 * This method sets the intake to a speed
 * 
 * @param speed
 *            IF YOU ARE A NEWBIE, USE HARDWARE.someJoystick.getY(), or
 *            something whatever
 *            NON-NEWBS - set the speed of the intake
 * 
 */
public void moveIntake (double speed)
{
    if (Math.abs(speed) > Math.abs(JOYSTICK_DEADBAND))
        {
        this.intakeMotor.set(speed);
        }
}

/**
 * Moves the arm to the desired position, 0 is the bottom, X is the top
 * 
 * NEWBIES CAN IGNORE
 * 
 * @param distance
 *            the desired distance the forklift goes in inches.
 * @param forkliftSpeed
 *            the speed of the forklift
 * @return true if the forklift is at or above the specified height, set the
 *         distance higher if you are going down from where the forklift was
 *         initially
 */
public boolean moveLiftDistance (double distance, double forkliftSpeed)
{
    double direction = distance - getForkliftHeight();
    boolean mustGoUp = direction > 1;

    if (mustGoUp == true)
        {
        this.forkliftMotor.set(forkliftSpeed);
        if (this.forkliftEncoder.getDistance() >= distance)
            {
            this.forkliftMotor.set(0);
            return true;
            }
        this.forkliftMotor.set(forkliftSpeed);
        return false;
        }
    this.forkliftMotor.set(-forkliftSpeed);
    if (this.forkliftEncoder.getDistance() <= distance)
        {
        this.forkliftMotor.set(0);
        return true;
        }
    this.forkliftMotor.set(-forkliftSpeed);
    return false;
}

/**
 * @return true if a cube has been scored in the switch
 */
public boolean scoreSwitch ()
{
    scoreSwitchState switchState = scoreSwitchState.MOVE_LIFT;
    switch (switchState)
        {
        case MOVE_LIFT:
            if (moveLiftDistance(SWITCH_HEIGHT, FORKLIFT_SPEED) == true)
                {
                switchState = scoreSwitchState.DEPLOY_INTAKE;
                }
            break;
        case DEPLOY_INTAKE:
            switchState = scoreSwitchState.SPIT_OUT_CUBE;
            break;
        case SPIT_OUT_CUBE:
            switchState = scoreSwitchState.FINISHED;
            break;
        case FINISHED:
            return true;
        }
    return false;
}

private enum scoreSwitchState
    {
MOVE_LIFT, DEPLOY_INTAKE, SPIT_OUT_CUBE, FINISHED
    }

/**
 * Stops the forklift motor and the intake motor
 */
// STOP STUFF
public void stopEverything ()
{
    this.forkliftMotor.set(0);
    this.intakeMotor.set(0);
}

/**
 * Stops the forklift
 */
public void stopForklift ()
{
    this.forkliftMotor.set(0);
}

/**
 * Stops the intake
 */
public void stopIntake ()
{
    this.intakeMotor.set(0);
}

private final double FORKLIFT_MAX_HEIGHT = 100;

private final double FORKLIFT_MIN_HEIGHT = 2;

private final double FORKLIFT_SPEED = .9;

private final double FORKLIFT_SPEED_COEFFICIENT = .9;

private final double INTAKE_SPEED = .5;

private final double SWITCH_HEIGHT = 30;

private final double INTAKE_ANGLE = 90;

private final double INTAKE_DEPLOY_SPEED = .9;

private final double JOYSTICK_DEADBAND = .2;
}
