package org.usfirst.frc.team339.Utils;

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

private double forkliftHeightForMoveLiftDistance = 0;

private boolean finishedForkliftMove = false;

// private boolean deployedArm = false;

private double forkliftSpeedUp = 0;

private double forkliftSpeedDown = 0;

// determines whether or not the intake motor should be stopped

private boolean stopIntake = false;


// the following booleans are for determining functions are using the intake

private boolean isRunningIntakeCube = false;

private boolean isRunningIntakeCubeOverride = false;

private boolean isRunningPushOutCubeAuto = false;

private boolean isRunningPushOutCubeTeleop = false;

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
 *
 * 
 * @param operatorJoystick
 *            Joystick object
 */
public void moveForkliftWithController (Joystick operatorJoystick)
{
    if (this.getForkliftHeight() <= FORKLIFT_MIN_HEIGHT + LIFT_TOLERANCE
            && this.liftState == forkliftState.MOVING_DOWN)
        {
        this.liftState = forkliftState.AT_STARTING_POSITION;
        }

    if (this.getForkliftHeight() >= FORKLIFT_MAX_HEIGHT
            && this.liftState == forkliftState.MOVING_UP)
        {
        this.liftState = forkliftState.STAY_AT_POSITION;
        }


    if (operatorJoystick.getY() <= -JOYSTICK_DEADBAND
            && this.getForkliftHeight() <= FORKLIFT_MAX_HEIGHT)
        {
        this.forkliftSpeedUp = FORKLIFT_SPEED_UP;
        this.liftState = forkliftState.MOVING_UP;
        }
    else if (operatorJoystick.getY() >= JOYSTICK_DEADBAND
            && this.getForkliftHeight() >= FORKLIFT_MIN_HEIGHT)
        {
        this.forkliftSpeedDown = FORKLIFT_SPEED_DOWN;
        this.liftState = forkliftState.MOVING_DOWN;
        }
    else if (this.getForkliftHeight() <= FORKLIFT_MIN_HEIGHT
            + LIFT_TOLERANCE)
        {
        this.liftState = forkliftState.AT_STARTING_POSITION;
        }
    else
        {
        this.liftState = forkliftState.STAY_AT_POSITION;
        }
}

/**
 * Intake runs until the light switch is on
 * 
 * To use this, just pass in a button value
 * 
 * Example of how to call this:
 * Hardware.cubeManipulator.intakeCube(Hardware.leftOperator.getRawButton(4));
 * 
 * DO NOT DO:
 * 
 * if(Hardware.leftOperator.getRawButton(4))
 * {
 * Hardware.cubeManipulator.intakeCube(Hardware.leftOperator.getRawButton(4));
 * }
 * 
 * @return true if the the cube is in, the light switch is off
 */
public boolean intakeCube (boolean button)
{
    if (button)
        {
        this.isRunningIntakeCube = true;

        if (this.intakeSwitch.isOn() == false)
            {
            this.intakeMotor.set(INTAKE_SPEED);
            return false;
            }
        this.intakeMotor.set(0);
        return true;
        }
    this.isRunningIntakeCube = false;
    return false;

}


/**
 * Intakes a cube and keeps going, even if the cube is already in there
 * 
 * To use this, just pass in a button value
 * 
 * Example of how to call this:
 * Hardware.cubeManipulator.intakeCubeOverride(Hardware.leftOperator.getRawButton(4));
 * 
 * DO NOT DO:
 * 
 * if(Hardware.leftOperator.getRawButton(4))
 * {
 * Hardware.cubeManipulator.intakeCubeOverride(Hardware.leftOperator.getRawButton(4));
 * }
 * 
 */
public void intakeCubeOverride (boolean button)
{
    if (button)
        {
        this.isRunningIntakeCubeOverride = true;
        this.intakeMotor.set(INTAKE_SPEED);
        }
    else
        {
        this.isRunningIntakeCubeOverride = false;
        }
}



/**
 * Pushes out the cube as long as a button is being held down
 * 
 * @param button-
 *            a button on a joystick
 */
public void pushOutCubeTeleop (boolean button)
{
    if (button)
        {
        this.isRunningPushOutCubeTeleop = true;
        this.intakeMotor.set(-INTAKE_SPEED);
        }
    else
        {
        this.isRunningPushOutCubeTeleop = false;
        }

}



/**
 * Pushes out the cube. For use in autonomous only
 * 
 * @return true if this function is complete, false if still going
 */
public boolean pushOutCubeAuto ()
{
    this.isRunningPushOutCubeAuto = true;
    switch (pushState)
        {
        case INIT:
            this.switchTimer.reset();
            this.switchTimer.start();
            this.pushState = pushOutState.PUSH_OUT;
            break;
        case PUSH_OUT:
            if (this.switchTimer.get() < EJECT_TIME)
                {
                this.intakeMotor.set(-this.INTAKE_SPEED);
                }
            else
                {
                this.switchTimer.stop();
                this.pushState = pushOutState.DONE;
                }
            break;
        case DONE:
            this.intakeMotor.set(0);
            this.pushState = pushOutState.INIT;
            this.isRunningPushOutCubeAuto = false;
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
 * Method for moving deploy arm to the down position; starts the deploy
 * arm
 * 
 * NEWBIES CAN IGNORE -- THIS ONLY APPLIES TO AUTO
 * 
 * @return true if the arm is down, false if it is still moving
 */
public boolean deployCubeIntake ()
{
    // advances the deploy intake state machine if it hasn't already been
    // deployed/ is deploying
    if (deployIntakeState == DeployState.INIT)
        {
        deployIntakeState = DeployState.DEPLOYING;
        }

    // returns whether or not the intake has finished deploying
    if (deployIntakeState == DeployState.FINISHED)
        {
        return true;
        }

    return false;

    // double degreesPerEncoderTick = 1.0;
    //
    // // if (this.intakeDeployEncoder.getDistance() <= INTAKE_ANGLE)
    // if (this.intakeDeployEncoder.get()
    // * degreesPerEncoderTick <= INTAKE_ANGLE)
    // {
    // this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
    // this.deployedArm = false;
    // }
    // else
    // {
    // this.intakeDeployMotor.set(0);
    // this.deployedArm = true;
    // // liftState = forkliftState.INTAKE_IS_DEPLOYED;
    // return true;
    // }
    // return false;
}

/**
 * Checks if the intake is deployed
 * 
 * @return true if the arm is deployed, and false if it isn't
 */
public boolean isIntakeDeployed ()
{
    return deployIntakeState == DeployState.FINISHED;
}


/**
 * Returns whether or not the cube intake mechanism is holding a cube
 * 
 * @return true if the photo switch is picking up a cube in our
 *         cube intake mechanism, false if otherwise
 */
public boolean hasCube ()
{
    return this.intakeSwitch.isOn();
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
 * 
 * @param distance
 *            the desired distance the forklift goes in inches.
 * @param forkliftSpeed
 *            the speed of the forklift ALWAYS USE POSITIVE
 * @return true if the forklift is at or above the specified height, set the
 *         distance higher if you are going down from where the forklift was
 *         initially
 */
public boolean moveLiftDistance (double distance, double forkliftSpeed)
{
    this.finishedForkliftMove = false;
    double direction = distance - getForkliftHeight();
    boolean mustGoUp = direction > 0;
    this.forkliftHeightForMoveLiftDistance = distance;
    if (mustGoUp == true && this
            .getForkliftHeight() <= this.forkliftHeightForMoveLiftDistance
                    + LIFT_TOLERANCE)
        {
        this.forkliftSpeedUp = -forkliftSpeed;
        this.liftState = forkliftState.MOVING_UP;
        return this.finishedForkliftMove;
        }
    else if (mustGoUp == false && this
            .getForkliftHeight() >= this.forkliftHeightForMoveLiftDistance
                    + LIFT_TOLERANCE)
        {
        this.forkliftSpeedDown = forkliftSpeed;
        this.liftState = forkliftState.MOVING_DOWN;
        }
    else
        {
        this.finishedForkliftMove = true;
        liftState = forkliftState.STAY_AT_POSITION;
        }
    return this.finishedForkliftMove;

}

/**
 * Moves the arm to the desired position, 0 is the bottom, X is the top
 * 
 * 
 * @param distance
 *            the desired distance the forklift goes in inches.
 * @return true if the forklift is at or above the specified height, set the
 *         distance higher if you are going down from where the forklift was
 *         initially
 */
public boolean moveLiftDistance (double distance)
{
    // TODO change using mustGoUp and set speed to forklift speed up of forklift
    // speed down constants
    if (this.getForkliftHeight() < distance)
        {
        moveLiftDistance(distance, FORKLIFT_SPEED_UP);
        }
    else
        {
        moveLiftDistance(distance, FORKLIFT_SPEED_DOWN);
        }
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
            if (moveLiftDistance(SWITCH_HEIGHT,
                    FORKLIFT_SPEED_UP) == true)
                {


                switchState = scoreSwitchState.DEPLOY_INTAKE;

                this.switchState = scoreSwitchState.DEPLOY_INTAKE;

                }
            break;
        case DEPLOY_INTAKE:
            System.out.println("Deploying intake");
            if (deployCubeIntake() == true)
                {

                System.out.println("Deployed intake");
                switchState = scoreSwitchState.SPIT_OUT_CUBE;

                this.switchState = scoreSwitchState.SPIT_OUT_CUBE;

                }
            break;
        case SPIT_OUT_CUBE:
            System.out.println("Spitting out cube");
            if (pushOutCubeAuto() == true)
                {

                System.out.println("Spat out cube");
                switchState = scoreSwitchState.FINISHED;

                this.switchState = scoreSwitchState.FINISHED;

                }
            break;
        case FINISHED:
            stopEverything();
            this.switchState = scoreSwitchState.MOVE_LIFT;
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
 * Scores a cube on a scale
 * 
 * @return true if a cube has been scored in the scale
 */
public boolean scaleSwitch ()
{
    System.out.println("started scale switch");
    switch (scaleState)
        {

        case DEPLOY_INTAKE:
            System.out.println("Deploying intake");
            if (deployCubeIntake() == true)
                {
                System.out.println("Deployed intake");
                scaleState = scoreScaleState.SPIT_OUT_CUBE;
                }
            break;
        case SPIT_OUT_CUBE:
            System.out.println("Spitting out cube");
            if (pushOutCubeAuto() == true)
                {
                System.out.println("Spat out cube");
                scaleState = scoreScaleState.FINISHED;
                }
            break;
        case FINISHED:
            stopEverything();
            // scaleState = scoreScaleState.MOVE_LIFT;
            return true;
        }

    return false;


}

scoreScaleState scaleState = scoreScaleState.DEPLOY_INTAKE;


private enum scoreScaleState
    {
DEPLOY_INTAKE, SPIT_OUT_CUBE, FINISHED
    }

/**
 * Stops the forklift motor and the intake motor
 */
// STOP STUFF
public void stopEverything ()
{
    this.liftState = forkliftState.STAY_AT_POSITION;
    this.intakeMotor.set(0);
}

/**
 * Keeps the forklift in the same position, stopping it from moving anymore
 */
public void stopForklift ()
{
    this.liftState = forkliftState.STAY_AT_POSITION;
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
    System.out.println("liftstate :" + liftState);
    switch (liftState)
        {
        case MOVING_UP:
            // System.out.println("TRYING TO GO UP");
            this.finishedForkliftMove = false;
            if (Math.abs(
                    this.getForkliftHeight()) >= FORKLIFT_MAX_HEIGHT)
                {
                this.liftState = forkliftState.STAY_AT_POSITION;
                }
            else
                {
                this.forkliftMotor.set(this.forkliftSpeedUp);
                }
            break;
        case MOVING_DOWN:
            // System.out.println("TRYING TO GO DOWN");
            if (this.getForkliftHeight() <= FORKLIFT_MIN_HEIGHT
                    + LIFT_TOLERANCE)
                {
                this.liftState = forkliftState.AT_STARTING_POSITION;
                this.finishedForkliftMove = true;
                }
            else
                {
                this.forkliftMotor.set(this.forkliftSpeedDown);
                }
            this.finishedForkliftMove = false;
            break;
        case STAY_AT_POSITION:
            // System.out.println("WE ARE STAYING AT POSITION");
            this.forkliftMotor.set(FORKLIFT_STAY_UP_SPEED);
            this.finishedForkliftMove = true;
            break;
        case AT_STARTING_POSITION:
            // System.out.println("WE ARE AT STARTING POSITION");
            this.forkliftMotor.set(FORKLIFT_AT_STARTING_POSITION);
            break;
        // this code isn't necessary because we made a seperate state
        // machine for the intake
        // case INTAKE_IS_DEPLOYED:
        // this.intakeMotor.set(0);
        // break;
        }

    // state machine for deploying the intake
    switch (deployIntakeState)
        {
        // initial state of the intake deploy motor; just stays still
        // until the deployCubeIntake() function is called
        case INIT:
            this.intakeDeployMotor.set(0.0);
            break;


        // moves the intake until the encoder reads that the arm has
        // turned the specified angle, then stops the intake deploy motor and
        // moves to the next state
        case DEPLOYING:
            this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
            if (this.intakeDeployEncoder.get() >= INTAKE_DEPLOY_ANGLE
                    - INTAKE_DEPLOY_COMPENSATION)
                {

                // stops the intake deploy motor if we've turned far enough;
                // FINISHED does this as well, but doing it here helps
                // keep the motor from overshooting too much
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.FINISHED;
                }
            break;

        // final state in the state machine; stops the intake deploy
        // motor
        case FINISHED:
            this.intakeDeployMotor.set(0.0);
            break;

        // we shouldn't ever get here, but in case we do, stop the intake
        // deploy motor
        default:
            this.intakeDeployMotor.set(0.0);
            break;
        }


    // checks if any of the specified functions wants to move the intake
    // motor
    this.stopIntake = !(isRunningIntakeCube
            || isRunningIntakeCubeOverride ||
            isRunningPushOutCubeAuto || isRunningPushOutCubeTeleop);

    // if none of the functions from above want to run the intake, then
    // stop the intake motor
    if (stopIntake == true)
        {
        this.stopIntake();
        }

}

private forkliftState liftState = forkliftState.AT_STARTING_POSITION;

// variable that controls the deploy intake state machine
private DeployState deployIntakeState = DeployState.INIT;

/**
 * @author Becky Button
 *
 */
public enum forkliftState
    {
MOVING_UP, MOVING_DOWN, STAY_AT_POSITION, AT_STARTING_POSITION, INTAKE_IS_DEPLOYED
    }

/**
 * Enum used for controlling the deploy intake state machine
 * 
 * @author Cole Ramos
 */
public static enum DeployState
    {
INIT, DEPLOYING, FINISHED
    }

private final double FORKLIFT_MAX_HEIGHT = 100;

private final double FORKLIFT_MIN_HEIGHT = 2;

private final double FORKLIFT_SPEED_UP = -.9;

private final double FORKLIFT_SPEED_DOWN = .4;




private final double FORKLIFT_STAY_UP_SPEED = 0.0;// -.15;


private final double FORKLIFT_AT_STARTING_POSITION = 0;

private final double INTAKE_SPEED = .5;

private final double SWITCH_HEIGHT = 30;


public final double SCALE_HEIGHT = 72;

private final double INTAKE_ANGLE = 90;

// how many degrees the intake deploy motor needs to turn for the intake
// to be fully deployed
private final double INTAKE_DEPLOY_ANGLE = 75;

// constant subtracted from the INTAKE_DEPLOY_ANGLE to help keep us
// from overshooting; needs to be tuned on the new robot
private final double INTAKE_DEPLOY_COMPENSATION = 0.0;


private final double INTAKE_DEPLOY_SPEED = .3;

private final double JOYSTICK_DEADBAND = .2;

private final double EJECT_TIME = 2.0;

private final double LIFT_TOLERANCE = 3;
}
