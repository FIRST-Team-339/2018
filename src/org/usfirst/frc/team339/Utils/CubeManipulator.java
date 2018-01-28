package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * Class for forklift and intake subsystems which includes as of 1/16/2018, an
 * intake, and an intake deploy thingy
 * 
 * @author Becky Button
 *
 */
public class CubeManipulator
{
private Victor forkliftMotor = null;

private Victor intakeMotor = null;

private Victor intakeDeployMotor = null;

private Encoder intakeDeployEncoder = null;

private LightSensor intakeSwitch = null;

private Encoder forkliftEncoder = null;

private Timer switchTimer = null;

private double forkliftHeight = 0;

private double forkliftSpeedForMoveDistance = 0.0;

private double forkliftSpeedDown = 0.4;

private double forkliftSpeedUp = -0.9;

private boolean finishedForkliftMove = false;

private boolean deployedArm = false;


/**
 * @param forkliftMotor
 * @param intakeMotor
 * @param intakeSwitch
 * @param forkliftEncoder
 * @param intakeDeploy
 * @param intakeDeployEncoder
 * @param timer
 */
public CubeManipulator (Victor forkliftMotor, Victor intakeMotor,
        LightSensor intakeSwitch, Encoder forkliftEncoder,
        Victor intakeDeploy, Encoder intakeDeployEncoder, Timer timer)
{
    this.forkliftMotor = forkliftMotor;
    this.intakeMotor = intakeMotor;
    this.intakeSwitch = intakeSwitch;
    this.forkliftEncoder = forkliftEncoder;
    this.intakeDeployMotor = intakeDeploy;
    this.intakeDeployEncoder = intakeDeployEncoder;
    this.switchTimer = timer;
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
    if (operatorJoystick.getY() <= -JOYSTICK_DEADBAND)
        {
        // this.forkliftHeight = FORKLIFT_MAX_HEIGHT;

        liftState = forkliftState.MOVING_UP;

        }
    else if (operatorJoystick.getY() >= JOYSTICK_DEADBAND)
        {
        // this.forkliftHeight = FORKLIFT_MIN_HEIGHT;
        // liftState = forkliftState.MOVING_DOWN;
        Hardware.liftingMotor.set(0.0);
        }
    else if (Hardware.liftingEncoder
            .getDistance() <= FORKLIFT_MIN_HEIGHT + 3)
        {
        liftState = forkliftState.AT_STARTING_POSITION;
        }
    else
        {
        liftState = forkliftState.STAY_AT_POSITION;
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
        this.deployedArm = false;
        return false;
        }
    this.intakeMotor.set(0);
    this.deployedArm = true;
    return true;
}

/**
 * NEWBIES USE ON A BUTTON!!!!
 * 
 * @return true if the complete
 */
public boolean pushOutCube ()
{
    switch (pushState)
        {
        case INIT:
            switchTimer.reset();
            switchTimer.start();
            pushState = pushOutState.PUSH_OUT;
            break;
        case PUSH_OUT:
            if (switchTimer.get() < EJECT_TIME)
                {
                this.intakeMotor.set(-this.INTAKE_SPEED);
                }
            else
                {
                switchTimer.stop();
                pushState = pushOutState.DONE;
                }
            break;
        case DONE:
            this.intakeMotor.set(0);
            pushState = pushOutState.INIT;
            return true;
        }
    return false;
}

private pushOutState pushState = pushOutState.INIT;

private static enum pushOutState
    {
INIT, PUSH_OUT, DONE
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
        this.deployedArm = false;
        }
    else
        {
        this.intakeDeployMotor.set(0);
        this.deployedArm = true;
        liftState = forkliftState.INTAKE_IS_DEPLOYED;
        return true;
        }
    return false;
}

/**
 * Checks if the intake is deployed
 * 
 * @return true if the arm is deployed, and false if it isn't
 */
public boolean isIntakeDeployed ()
{
    return deployedArm;
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
    // finishedForkliftMove = false;
    double direction = distance - getForkliftHeight();
    boolean mustGoUp = direction > 1;
    this.forkliftHeight = distance;
    this.forkliftSpeedForMoveDistance = forkliftSpeed;
    if (mustGoUp == true)
        {
        liftState = forkliftState.MOVING_UP;
        return this.finishedForkliftMove;
        }
    liftState = forkliftState.MOVING_DOWN;
    return this.finishedForkliftMove;

}

/**
 * Scores a cube on a switch - NEWBIES - SET THIS TO A BUTTON
 * 
 * @return true if a cube has been scored in the switch
 */
public boolean scoreSwitch ()
{
    switch (switchState)
        {
        case MOVE_LIFT:
            if (moveLiftDistance(SWITCH_HEIGHT, FORKLIFT_SPEED) == true)
                {
                switchState = scoreSwitchState.DEPLOY_INTAKE;
                }
            break;
        case DEPLOY_INTAKE:
            if (deployCubeIntake() == true)
                {
                switchState = scoreSwitchState.SPIT_OUT_CUBE;
                }
            break;
        case SPIT_OUT_CUBE:
            pushOutCube();
            switchState = scoreSwitchState.FINISHED;

            break;
        case FINISHED:
            stopEverything();
            switchState = scoreSwitchState.MOVE_LIFT;
            return true;
        }

    return false;
}

scoreSwitchState switchState = scoreSwitchState.MOVE_LIFT;

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
    liftState = forkliftState.STAY_AT_POSITION;
    this.intakeMotor.set(0);
}

/**
 * Keeps the forklift in the same position, stopping it from moving anymore
 */
public void stopForklift ()
{
    liftState = forkliftState.STAY_AT_POSITION;
}

/**
 * Stops the intake
 */
public void stopIntake ()
{
    this.intakeMotor.set(0);
}

/**
 * Call this in periodic.
 * State machine for the forklift
 */
public void forkliftUpdate ()
{
    switch (liftState)
        {
        case MOVING_UP:
            System.out.println("TRYING TO GO UP");
            finishedForkliftMove = false;
            if (Math.abs(this.forkliftEncoder
                    .getDistance()) >= FORKLIFT_MAX_HEIGHT)
                {
                liftState = forkliftState.STAY_AT_POSITION;
                finishedForkliftMove = true;
                }
            else
                {
                this.forkliftMotor.set(this.forkliftSpeedUp);
                }
            break;
        case MOVING_DOWN:
            System.out.println("TRYING TO GO DOWN");
            if (Hardware.liftingEncoder
                    .getDistance() <= FORKLIFT_MIN_HEIGHT + 3)
                {
                liftState = forkliftState.AT_STARTING_POSITION;
                finishedForkliftMove = true;
                }
            else
                {
                this.forkliftMotor.set(this.forkliftSpeedDown);
                }
            finishedForkliftMove = false;
            break;
        case STAY_AT_POSITION:
            System.out.println("WE ARE STAYING AT POSITION");
            this.forkliftMotor.set(FORKLIFT_STAY_UP_SPEED);
            break;
        case AT_STARTING_POSITION:
            System.out.println("WE ARE AT STARTING POSITION");
            this.forkliftMotor.set(0.0);
            break;
        case INTAKE_IS_DEPLOYED:
            break;
        }
}

private forkliftState liftState = forkliftState.AT_STARTING_POSITION;

/**
 * @author Becky Button
 *
 */
public enum forkliftState
    {
MOVING_UP, MOVING_DOWN, STAY_AT_POSITION, AT_STARTING_POSITION, INTAKE_IS_DEPLOYED

    }

private final double FORKLIFT_MAX_HEIGHT = 100;

private final double FORKLIFT_MIN_HEIGHT = 2;

private final double FORKLIFT_SPEED = .9;

private final double FORKLIFT_SPEED_COEFFICIENT = .9;

private final double FORKLIFT_STAY_UP_SPEED = -0.15;

private final double INTAKE_SPEED = .5;

private final double SWITCH_HEIGHT = 30;

private final double INTAKE_ANGLE = 90;

private final double INTAKE_DEPLOY_SPEED = .9;

private final double JOYSTICK_DEADBAND = .2;

private final double EJECT_TIME = 2;
}
