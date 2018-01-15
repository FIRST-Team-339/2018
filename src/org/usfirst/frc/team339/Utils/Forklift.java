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
    return forkliftEncoder.getDistance();
}

/**
 * 
 * @param operatorJoystick
 *            Joystick object
 */
public void moveLiftWithController (Joystick operatorJoystick)
{
    if (getForkliftHeight() < FORKLIFT_MAX_HEIGHT
            || getForkliftHeight() > FORKLIFT_MIN_HEIGHT)
        {
        forkliftMotor.set(
                (operatorJoystick.getY()) * FORKLIFT_SPEED_COEFFICIENT);
        }
    else
        {
        forkliftMotor.set(0);
        }
}

/**
 * @return
 */
public boolean intakeCube ()
{
    if (intakeSwitch.isOn() == false)
        {
        intakeMotor.set(INTAKE_SPEED);
        return false;
        }
    intakeMotor.set(0);
    return true;
}

/**
 * @return
 */
public boolean pushOutCube ()
{
    if (intakeSwitch.isOn() == true)
        {
        this.intakeMotor.set(-this.INTAKE_SPEED);
        return false;
        }
    this.intakeMotor.set(0);
    return true;
}

public boolean deployCubeIntake ()
{
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
    this.intakeMotor.set(speed);
}

/**
 * moves the arm to the desired position
 * 
 * @param distance
 *            the desired distance the forklift goes in inches.
 * @return true if the forklift is at or above the specified height
 */
public boolean moveLiftDistance (double distance)
{
    double direction = distance - getForkliftHeight();
    boolean mustGoUp = direction > 1;
    // if (getForkliftHeight() < distance)
    // {
    // forkliftMotor.set(FORKLIFT_SPEED);
    // return false;
    // }
    // else if ()
    // {
    // forkliftMotor.set(0);
    return true;
    // }
}

public boolean scoreSwitch ()
{
    scoreSwitchState switchState = scoreSwitchState.MOVE_LIFT;
    switch (switchState)
        {
        case MOVE_LIFT:
            if (moveLiftDistance(SWITCH_HEIGHT) == true)
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

private final double FORKLIFT_UPPER_TOLERANCE = 1.5;

private final double INTAKE_SPEED = .5;

private final double SWITCH_HEIGHT = 30;
}
