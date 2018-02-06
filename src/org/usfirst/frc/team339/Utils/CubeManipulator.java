package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class for forklift and intake subsystems which includes a Forklift,
 * intake motors, and an intake deploy mechanism.
 * 
 * @author Becky Button
 *
 */
public class CubeManipulator
{
// ---------------HARDWARE---------------------

// ================FORKLIFT================
private SpeedController forkliftMotor = null;

private Encoder forkliftEncoder = null;
// ========================================

// ================INTAKE==================
private SpeedController intakeMotor = null;

private SpeedController intakeDeployMotor = null;

private Encoder intakeDeployEncoder = null;

private LightSensor intakeSwitch = null;

private Timer switchTimer = null;
// ========================================

// --------------------------------------------

/**
 * Creates the CubeManipulator class, which conrols the movement of the
 * forklift, intake, and climbing for autonomous and teleop functions
 * 
 * @param forkliftMotor
 *            Motor controller that powers the up and down movement of the
 *            forklift
 * @param intakeMotor
 *            Motor controller that powers the spinning wheels that grabs the
 *            cube
 * @param intakeSwitch
 *            Light sensor that senses whether or not we are in control of a
 *            cube
 * @param forkliftEncoder
 *            Sensor for software stops on the forklift
 * @param intakeDeploy
 *            Motor controller that is used to deploy the intake
 * @param intakeDeployEncoder
 *            Sensor for the software stops on the deploy motor
 * @param timer
 *            Timer used for the intake mechanism
 */
public CubeManipulator (SpeedController forkliftMotor,
        SpeedController intakeMotor, LightSensor intakeSwitch,
        Encoder forkliftEncoder, SpeedController intakeDeploy,
        Encoder intakeDeployEncoder, Timer timer)
{
    this.forkliftMotor = forkliftMotor;
    this.intakeMotor = intakeMotor;
    this.intakeSwitch = intakeSwitch;
    this.forkliftEncoder = forkliftEncoder;
    this.intakeDeployMotor = intakeDeploy;
    this.intakeDeployEncoder = intakeDeployEncoder;
    this.switchTimer = timer;
}

// ========================FORKLIFT FUNCTIONS========================

/**
 * Keeps the forklift in the same position, stopping it from moving anymore
 */
public void stopForklift ()
{
    this.liftState = forkliftState.STAY_AT_POSITION;
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
 * Moves the forklift up and down based on joystick input, for teleop.
 * 
 * @param operatorJoystick
 *            the joystick used when operting the forklift
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
 * Moves the arm to the desired position, FORKLIFT_MIN_HEIGHT is the bottom,
 * FORKLIFT_MAX_HEIGHT is the top
 * 
 * @param position
 *            The position that the forklift will be set to move to
 * @param forkliftSpeed
 *            how fast the robot should move it's forklift (0.0 to 1.0)
 * @return true if the forklift is at or above the specified height,
 *         false if we are still moving
 */
public boolean setLiftPosition (double position, double forkliftSpeed)
{
    this.finishedForkliftMove = false;
    double direction = position - getForkliftHeight();
    boolean mustGoUp = direction > 0;
    this.forkliftHeightForMoveLiftDistance = position;
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
 * Moves the arm to the the position input, FORKLIFT_MAX_HEIGHT being the
 * top soft stop, and FORKLIFT_MIN_HEIGHT being the FORKLIFT_MIN_HEIGHT.
 * 
 * Overloads the setLiftPosition, using the FORKLIFT_SPEED constants
 * 
 * @param position
 *            The position the forklift will move to, in inches.
 * @return true if the forklift is at or above the specified height, false
 *         if still moving
 */
public boolean setLiftPosition (double position)
{
    if (this.getForkliftHeight() < position)
        {
        setLiftPosition(position, Math.abs(FORKLIFT_SPEED_UP));
        }
    else
        {
        setLiftPosition(position, Math.abs(FORKLIFT_SPEED_DOWN));
        }
    return this.finishedForkliftMove;
}

/**
 * For use in teleop and autonomous periodic.
 * 
 * Any functions that move the lift will NOT WORK UNLESS this function is called
 * as well.
 * 
 * Runs the forklift movement code in the background, which allows multiple
 * movements in autonomous state machines.
 */
public void forkliftUpdate ()
{
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

// ========================INTAKE FUNCTIONS========================

/**
 * Cuts power to the intake motors
 */
public void stopIntake ()
{
    this.intakeMotor.set(0);
}

/**
 * Sets the intake into the down position, which is required for the
 * robot to be able to grab any cubes, and drop them off.
 * 
 * REQUIRED be called before teleop periodic.
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
}

/**
 * Checks whether the intake is deployed
 * 
 * @return true if the arm is deployed, and false if it isn't
 */
public boolean isIntakeDeployed ()
{
    return deployIntakeState == DeployState.FINISHED;
}

/**
 * Moves the intake based on the speed provided.
 * 
 * @param speed
 *            How fast the motors should move, in percent. (-1.0 to 1.0)
 *            Negative is pushing out, positive is pulling in
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
 * Intakes the cube if the light sensor finds we don't have a cube. If we do,
 * then set it to 0.
 * 
 * Instead of having an if() statement, input the button.
 * 
 * @return true if we are in control of a cube
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
 * Scores a cube on a switch - NEWBIES - SET THIS TO A BUTTON
 * 
 * @return true if a cube has been scored in the switch
 */
public boolean scoreSwitch ()
{
    switch (switchState)
        {
        case MOVE_LIFT:
            if (setLiftPosition(SWITCH_HEIGHT,
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

/**
 * Scores a cube on a scale
 * 
 * @return true if a cube has been scored in the scale
 * 
 */
public boolean scoreScale ()
{
    switch (scaleState)
        {
        case MOVE_LIFT:
            System.out.println("Moving lift");
            if (setLiftPosition(SCALE_HEIGHT,
                    FORKLIFT_SPEED_UP) == true)
                {
                switchState = scoreSwitchState.DEPLOY_INTAKE;
                this.switchState = scoreSwitchState.DEPLOY_INTAKE;
                }
            break;
        case DEPLOY_INTAKE:
            System.out.println("Deploying Intake");
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
                scaleState = scoreScaleState.FINISHED;
                }
            break;
        case FINISHED:
            System.out.println("Finished");
            stopEverything();
            scaleState = scoreScaleState.SPIT_OUT_CUBE;
            return true;
        }
    return false;
}

/**
 * Stops the forklift motor and the intake motor
 */
public void stopEverything ()
{
    stopForklift();
    stopIntake();
}



/**
 * List of states used when updating the forklift position
 * 
 * @author Becky Button
 */
private static enum forkliftState
    {
MOVING_UP, MOVING_DOWN, STAY_AT_POSITION, AT_STARTING_POSITION, INTAKE_IS_DEPLOYED
    }

/**
 * Enum used for controlling the deploy intake state machine
 * 
 * @author Cole Ramos
 */
private static enum DeployState
    {
INIT, DEPLOYING, FINISHED
    }

/**
 * List of states used when ejecting the cube autonomously
 */
private static enum pushOutState
    {
INIT, PUSH_OUT, DONE
    }

/**
 * List of states used when scoring a cube on the switch
 */
private static enum scoreSwitchState
    {
MOVE_LIFT, DEPLOY_INTAKE, SPIT_OUT_CUBE, FINISHED
    }

/**
 * List of states used when scoring a cube on the switch
 */
private static enum scoreScaleState
    {
MOVE_LIFT, DEPLOY_INTAKE, SPIT_OUT_CUBE, FINISHED
    }

// --------------------VARIABLES--------------------
// ================FORKLIFT================
private forkliftState liftState = forkliftState.AT_STARTING_POSITION;

private double forkliftHeightForMoveLiftDistance = 0;

private boolean finishedForkliftMove = false;

private double forkliftSpeedUp = 0;

private double forkliftSpeedDown = 0;
// ========================================

// ================INTAKE==================
// private boolean deployedArm = false;

// variable that controls the deploy intake state machine
private DeployState deployIntakeState = DeployState.INIT;

private pushOutState pushState = pushOutState.INIT;

// determines whether or not the intake motor should be stopped
private boolean stopIntake = false;

// the following booleans are for determining functions are using the intake

private boolean isRunningIntakeCube = false;

private boolean isRunningIntakeCubeOverride = false;

private boolean isRunningPushOutCubeAuto = false;

private boolean isRunningPushOutCubeTeleop = false;
// ========================================

private scoreScaleState scaleState = scoreScaleState.SPIT_OUT_CUBE;

private scoreSwitchState switchState = scoreSwitchState.MOVE_LIFT;

// -------------------------------------------------

// --------------------CONSTANTS--------------------

// ================FORKLIFT================
private final double FORKLIFT_MAX_HEIGHT = 100;

private final double FORKLIFT_MIN_HEIGHT = 2;

private final double FORKLIFT_SPEED_UP = -.9;

private final double FORKLIFT_SPEED_DOWN = .4;

private final double FORKLIFT_AT_STARTING_POSITION = 0;

private final double FORKLIFT_STAY_UP_SPEED = 0.0;// -.15;

private final double LIFT_TOLERANCE = 3;

private final double SWITCH_HEIGHT = 30;

public final double SCALE_HEIGHT = 72;
// =========================================

// ================INTAKE===================
private final double INTAKE_SPEED = .5;

// how many degrees the intake deploy motor needs to turn for the intake
// to be fully deployed
private final double INTAKE_DEPLOY_ANGLE = 75;

private final double INTAKE_ANGLE = 90;

// constant subtracted from the INTAKE_DEPLOY_ANGLE to help keep us
// from overshooting; needs to be tuned on the new robot
private final double INTAKE_DEPLOY_COMPENSATION = 0.0;

private final double INTAKE_DEPLOY_SPEED = .3;

private final double EJECT_TIME = 2.0;
// =========================================

private final double JOYSTICK_DEADBAND = .2;

// ---------------------------------------------
}
