package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    // System.out.println("Calling stop forklift");
    this.liftState = ForkliftState.STAY_AT_POSITION;
}

/**
 * Gets the value from the encoder, which is the absolute position above the
 * ground.
 * 
 * @return the height of the forklift, in inches
 */
public double getForkliftHeight ()
{
    return this.forkliftEncoder.getDistance();
}

/**
 * Moves the forklift up and down based on joystick input, for teleop.
 * 
 * @param overrideButton
 *            the button that, if helf, activates forklift override
 * @param speed
 *            How fast the forklift should be moving, in percent.
 * @author C.R.
 * 
 */
public void moveForkliftWithController (double speed,
        boolean overrideButton)
{
    // Override button, ignore encoder.
    if (overrideButton == true)
        {
        this.forkliftTargetSpeed = speed;
        this.liftState = ForkliftState.MOVE_JOY;
        }
    else
        {
        // If we are past the max height or below the min, don't move the
        // motors.
        if ((speed > 0
                && forkliftEncoder.getDistance() > FORKLIFT_MAX_HEIGHT)
                || (speed < 0 && forkliftEncoder
                        .getDistance() < currentMinLiftPosition)
                || Math.abs(speed) < JOYSTICK_DEADBAND)
            return;
        // Move the forklift the desired speed
        forkliftTargetSpeed = speed;
        this.liftState = ForkliftState.MOVE_JOY;
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
    // Sets the target position and speed, enables "moving-to-position" state.
    if (setLiftPositionInit == true)
        {
        forkliftTargetHeight = position;
        forkliftTargetSpeed = Math.abs(forkliftSpeed);

        liftState = ForkliftState.MOVING_TO_POSITION;
        setLiftPositionInit = false;
        }
    // return true is we are done moving, false is we are still going
    if (liftState == ForkliftState.STAY_AT_POSITION)
        {
        setLiftPositionInit = true;
        return true;
        }
    return false;
}

private boolean setLiftPositionInit = true;

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
    double defaultSpeed = 0.0;
    // If the requested position is greater than the current position, set the
    // state machine to go up.
    if (this.getForkliftHeight() < position)
        {
        defaultSpeed = FORKLIFT_DEFAULT_SPEED_UP;
        }
    // Else, we are going down.
    else
        {
        defaultSpeed = FORKLIFT_DEFAULT_SPEED_DOWN;
        }

    return setLiftPosition(position, defaultSpeed);
}

// ========================INTAKE FUNCTIONS========================

/**
 * Get the angle (in ticks) that the intakeDeployEncoder is reading
 * 
 * @return the ticks of the intakeDeployEncoder
 */
public double getIntakeAngle ()
{
    return this.intakeDeployEncoder.get();
}

/**
 * Tell the intake motor to stop by setting it to zero. Should be changed to
 * private because NO ONE SHOULD BE USING THIS OUTSIDE OF THE STATE MACHINE
 */
private void stopIntake ()
{
    this.intakeMotor.set(0);
}

/**
 * Sets the intake into the down position, which is required for the
 * robot to be able to grab any cubes, and drop them off.
 * 
 * Only works if the intake mechanism is not currently already deployed
 * 
 * REQUIRED to be called before teleop periodic.
 * 
 * If the override boolean for this function is true, will call the override
 * code to always move the deployIntake down
 * 
 * @param override
 *            Whether or not we want to ignore the encoder
 * 
 * @return true if the arm is down, false if it is still moving
 */
public boolean deployCubeIntake (boolean override)
{
    if (override)
        {
        deployIntakeState = DeployState.OVERRIDE_DEPLOY;
        return true;
        }
    // advances the deploy intake state machine if it hasn't already been
    // deployed/ is deploying
    if (deployIntakeState == DeployState.NOT_DEPLOYED)
        {
        deployIntakeState = DeployState.DEPLOYING;
        }

    // returns whether or not the intake has finished deploying
    if (deployIntakeState == DeployState.DEPLOYED)
        {
        return true;
        }

    return false; // returns false if we haven't finished deploying
}


/**
 * Draws the cube intake mechanism back into the robot if it the mechanism
 * is not already deployed
 * 
 * @param override
 *            Whether or not we want to ignore the encoder.
 * @return true if the mechanism is retracted/ in the robot, false is otherwise
 *         (hasn't finished retracting)
 * 
 * @author Cole Ramos
 * 
 */
public boolean retractCubeIntake (boolean override)
{
    if (override)
        {
        deployIntakeState = DeployState.OVERRIDE_RETRACT;
        return true;
        }
    // tells the deployIntake state machine to retract if the cube intake
    // mechanism was already deployed
    if (deployIntakeState == DeployState.DEPLOYED)
        {
        deployIntakeState = DeployState.RETRACTING;
        }

    // returns true if it has finished retracting or was not deployed in the
    // first place
    if (deployIntakeState == DeployState.NOT_DEPLOYED)
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
    return deployIntakeState == DeployState.DEPLOYED;
}


/**
 * Intakes the cube if the light sensor finds we don't have a cube. If we do,
 * then set it to 0.
 * 
 * Instead of having an if() statement, input the button.
 * 
 * @param button
 *            Button/buttons used when intaking a cube
 * @param pullInOverride
 *            Override if the cubeSensor stops working, for pulling in the cube
 * @param pushOutOverride
 *            Override if the cubeSensor stops working, for pushing out the cube
 */
public void intakeCube (boolean button, boolean pullInOverride,
        boolean pushOutOverride)
{
    // Override the 'pull in', ignoring the photoswitch
    if (pullInOverride == true)
        {
        intakeState = IntakeState.PULL_IN;
        intakeOverride = true;
        return;
        }
    // Override the 'push out', ignoring the photoswitch
    if (pushOutOverride == true)
        {
        intakeState = IntakeState.PUSH_OUT;
        return;
        }
    // Only run the override if the button is pressed
    intakeOverride = false;

    // If it's the buttons first run, then figure out if we will be pulling the
    // cube in or pushing it out
    if (button == true && lastIntakeButtonStatus == false)
        {
        isPullingIn = hasCube() == false;
        }
    // If it's not the buttons first run but the button is still being pressed,
    // then continue what we were doing previously.
    else if (button == true)
        {
        if (isPullingIn == true)
            intakeState = IntakeState.PULL_IN;
        else
            intakeState = IntakeState.PUSH_OUT;
        }

    // Reset the 'button's first run?' status
    lastIntakeButtonStatus = button;

}

/**
 * Pushes out the cube. For use in autonomous only
 * 
 * @return true if this function is complete, false if still going
 */
public boolean pushOutCubeAuto ()
{
    // Tell the state machine to stop controlling the intake motors
    this.isRunningPushOutCubeAuto = true;

    switch (pushState)
        {
        // Reset timer
        case INIT:
            this.switchTimer.reset();
            this.switchTimer.start();
            this.pushState = pushOutState.PUSH_OUT;
            break;
        // Push out cube for EJECT_TIME seconds
        case PUSH_OUT:
            // EJECT_TIME seconds has not elapsed? run motors.
            if (this.switchTimer.get() < EJECT_TIME)
                {
                this.intakeMotor.set(-this.INTAKE_SPEED);
                }
            // Time has elapsed? stop timer and move to next state.
            else
                {
                this.switchTimer.stop();
                this.pushState = pushOutState.DONE;
                }
            break;
        // Stop motors and reset state machine for future uses.
        default:
            System.out.println("Error finding state " + pushState
                    + " in CubeManipulator.pushOutCubeAuto()");
        case DONE:
            this.intakeMotor.set(0);
            this.pushState = pushOutState.INIT;
            this.isRunningPushOutCubeAuto = false;
            return true;

        }
    // we have NOT finished pushing out the cube.
    return false;
}

/**
 * If the cube sensor is triggered, then there should be a cube inside the
 * manipulator
 * 
 * @return Whether or not the cube sensor is triggered.
 */
public boolean hasCube ()
{
    return this.intakeSwitch.isOn();
}



// TODO make sure this works
/**
 * Scores a cube on a scale autonomously, using the forklift, intake and intake
 * deploy.
 * 
 * Not currently working
 * 
 * @return true if a cube has been scored in the scale
 */
public boolean scoreScale ()
{
    System.out.println("scoring on the scale");
    switch (scaleState)
        {
        // If the intake has not been deployed already, then do so.
        case DEPLOY_INTAKE:
            System.out.println("Deploying intake");
            if (deployCubeIntake(false) == true)
                {
                scaleState = scoreScaleState.MOVE_LIFT;
                }
            break;
        // Move the lift to the scale height, and move on when it's finished
        case MOVE_LIFT:
            System.out.println("forklift hight:"
                    + Hardware.cubeManipulator.getForkliftHeight());
            System.out.println("Moving lift");
            if (setLiftPosition(SCALE_HEIGHT,
                    FORKLIFT_DEFAULT_SPEED_UP) == true)
                {
                scaleState = scoreScaleState.SPIT_OUT_CUBE;
                }
            break;
        // Eject the cube (onto the scale preferably)
        case SPIT_OUT_CUBE:
            System.out.println("Spitting out cube");
            if (pushOutCubeAuto() == true)
                {
                scaleState = scoreScaleState.FINISHED;
                }
            break;
        // If we have an undefined state input, for debugging
        default:
            System.out.println("Error finding state " + scaleState
                    + " in CubeManipulator.scoreScale()");
            // We have finished (hopefully) scoring on the scale! Hurrah!
        case FINISHED:
            stopEverything();
            this.scaleState = scoreScaleState.MOVE_LIFT;
            return true;
        }
    // We have not yet finished scoring the scale
    return false;
}



// ===================== Update Methods ========================

/**
 * Master update function that calls the update functions for forklift,
 * deploy intake, and intake/ push out cube, allowing them to properly
 * use their state machines
 * 
 * @author Cole Ramos
 * 
 */
public void masterUpdate ()
{

    // update the forklift state machine
    forkliftUpdate();
    // update the deployIntake state machine
    deployIntakeUpdate();
    // update the intake/ PushOut cube state machines
    intakeUpdate();
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
    SmartDashboard.putString("Forklift Update", liftState.toString());

    // If we don't have a cube or the override is enabled, then set the min to
    // the lower position, to grab cubes.
    if (hasCube() == false || this.intakeOverride == true)
        {
        this.currentMinLiftPosition = FORKLIFT_NO_CUBE_MIN_HEIGHT;
        }
    // If we have a cube set the minimum to higher, to make sure we don't drag
    // the cube on the ground.
    else
        {
        this.currentMinLiftPosition = FORKLIFT_WITH_CUBE_MIN_HEIGHT;
        }

    // main switch statement for the forklift state machine
    switch (liftState)
        {
        case MOVING_TO_POSITION:
            // Make sure we don't move past the MAX or MIN position
            if ((this.forkliftTargetHeight > FORKLIFT_MAX_HEIGHT
                    && forkliftTargetHeight > forkliftEncoder
                            .getDistance())
                    || (this.forkliftTargetHeight < currentMinLiftPosition
                            && forkliftTargetHeight < forkliftEncoder
                                    .getDistance()))
                {
                liftState = ForkliftState.STAY_AT_POSITION;
                break;
                }

            // Begins by stating whether we are increasing or decreasing
            if (forkliftDirection == ForkliftDirectionState.NEUTRAL)
                {
                if (forkliftTargetHeight < forkliftEncoder
                        .getDistance())
                    forkliftDirection = ForkliftDirectionState.MOVING_DOWN;
                else
                    forkliftDirection = ForkliftDirectionState.MOVING_UP;
                }

            // Differentiate moving up from down
            if (forkliftDirection == ForkliftDirectionState.MOVING_UP)
                {
                // If we have passed the value we wanted...
                if (this.forkliftEncoder
                        .getDistance() > forkliftTargetHeight)
                    {
                    liftState = ForkliftState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    forkliftDirection = ForkliftDirectionState.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value, keep going up.
                this.forkliftMotor.set(forkliftTargetSpeed);
                }
            else
                {
                // If we have passed the value we wanted...
                if (this.forkliftEncoder
                        .getDistance() < forkliftTargetHeight)
                    {
                    liftState = ForkliftState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    forkliftDirection = ForkliftDirectionState.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value, keep going down.
                this.forkliftMotor.set(-forkliftTargetSpeed);
                }

            break;
        case MOVE_JOY:
            setLiftPositionInit = true;
            this.forkliftMotor.set(forkliftTargetSpeed);
            // IF we are no longer holding the joystick, then it will
            // automatically stay at position.
            liftState = ForkliftState.STAY_AT_POSITION;
            break;
        default:
            // print out we reached the default case (which we shouldn't have),
            // then fall through to STAY_AT_POSITION
            System.out.println(
                    "Reached default in the liftState switch in "
                            + "forkliftUpdate in CubeManipulator");
        case STAY_AT_POSITION:
            // IF we have a cube, then send a constant voltage.
            if (this.hasCube() == true)
                {
                this.forkliftMotor.set(FORKLIFT_STAY_UP_WITH_CUBE);
                }
            else
                {
                this.forkliftMotor.set(FORKLIFT_STAY_UP_SPEED);
                }
            // Reset the direction for next move-to-position.
            forkliftDirection = ForkliftDirectionState.NEUTRAL;
            setLiftPositionInit = true;
        }
}



/**
 * Update method for the deployIntake functions. Allows the deployIntake
 * code to use their state machine. deployIntake and related functions
 * will not work unless this updateFunction is called
 * 
 * @author Cole Ramos
 * @edited Ryan McGee
 * 
 */
public void deployIntakeUpdate ()
{

    // state machine for deploying the intake
    switch (deployIntakeState)
        {
        // initial state of the intake deploy motor; just stays still
        // until the deployCubeIntake() function is called
        case NOT_DEPLOYED:
            this.intakeDeployMotor.set(0.0);
            break;

        // moves the intake until the encoder reads that the arm has
        // turned the specified angle, then stops the intake deploy motor
        // and
        // moves to the next state
        case DEPLOYING:

            this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);

            if (this.getIntakeAngle() >= INTAKE_DEPLOY_TICKS)
                {
                // stops the intake deploy motor if we've turned far enough;
                // FINISHED does this as well, but doing it here helps
                // keep the motor from overshooting too much
                this.setLiftPosition(FORKLIFT_WITH_CUBE_MIN_HEIGHT);
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.DEPLOYED;
                }
            break;

        // final state in the state machine; stops the intake deploy
        // motor
        case DEPLOYED:
            this.intakeDeployMotor.set(0.0);
            break;


        // brings the intake mechanism back into the robot, and sets the
        // state to NOT_DEPLOYED
        case RETRACTING:
            this.intakeDeployMotor.set(INTAKE_RETRACT_SPEED);
            if (this.intakeDeployEncoder
                    .get() <= INTAKE_RETRACT_TICKS)
                {
                // brings back in the intake mechanism until the intake
                // deploy
                // encoder reads the INTAKE_RETRACT_ANGLE (usually 0)
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.NOT_DEPLOYED;
                }
            break;

        case OVERRIDE_DEPLOY:
            this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
            // If the override is let go, then stop the deploy
            deployIntakeState = DeployState.STOPPED;
            break;
        case OVERRIDE_RETRACT:
            this.intakeDeployMotor.set(INTAKE_RETRACT_SPEED);
            // If the override is let go, then stop the deploy
            deployIntakeState = DeployState.STOPPED;
            break;
        default:
            System.out.println(
                    "Unkown case found in deployIntakeUpdate(). Stopping deploy.");
        case STOPPED:
            this.intakeDeployMotor.stopMotor();
            break;
        }

}

/**
 * Update method for the intakeCube, pushOutCube, and related functions.
 * 
 * intakeCube and pushOutCube will not work unless this function is constantly
 * called
 * 
 * @author Cole Ramos
 * @edited Ryan McGee
 */
public void intakeUpdate ()
{
    // If we are autonomously pushing out the cube, then don't run this.
    if (isRunningPushOutCubeAuto == true)
        {
        isRunningPushOutCubeAuto = false;
        return;
        }

    switch (intakeState)
        {
        case PULL_IN:
            // We have a cube? stop pulling in.
            if (hasCube() == true && intakeOverride == false)
                this.intakeMotor.stopMotor();
            // Don't have a cube? keep pulling in.
            else
                this.intakeMotor.set(INTAKE_SPEED);

            // Set to stop when they stop hitting the button.
            intakeState = IntakeState.STOP;
            break;
        case PUSH_OUT:
            this.intakeMotor.set(-INTAKE_SPEED);
            // Set to stop when they stop hitting the button.
            intakeState = IntakeState.STOP;
            break;
        default:
            System.out.println("Unknown case in intakeUpdate()");
        case STOP:
            // If we have a cube, send a constant voltage to make sure it
            // doesn't come out.
            if (hasCube() == true)
                {
                this.intakeMotor.set(INTAKE_STOP_WITH_CUBE);
                }
            else
            // No cube? stop the motor.
                {
                this.intakeMotor.stopMotor();
                }
            break;
        }
}

// ===================== End Update Methods Section ========================


/**
 * Cuts the power to all motors.
 */
public void stopEverything ()
{
    stopForklift();
    stopIntake();
    intakeDeployMotor.stopMotor();
}



/**
 * List of states used when updating the forklift position
 * 
 * @author Becky Button
 * @edited C.R. and Ryan McGee
 */
private static enum ForkliftState
    {
MOVING_TO_POSITION, STAY_AT_POSITION, MOVE_JOY
    }

private static enum IntakeState
    {
BEGIN_MOVE, PUSH_OUT, PULL_IN, STOP
    }

/**
 * States used inside the MOVNG_TO_POSITION state (from ForkliftState) for
 * representing which way the forklift is moving
 * 
 * @author C.R.
 *
 */
private static enum ForkliftDirectionState
    {
MOVING_UP, MOVING_DOWN, NEUTRAL
    }

/**
 * Enum used for controlling the deploy intake state machine
 * 
 * @author Cole Ramos
 */
private static enum DeployState
    {
NOT_DEPLOYED, DEPLOYING, DEPLOYED, RETRACTING, OVERRIDE_DEPLOY, OVERRIDE_RETRACT, STOPPED
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
private static enum scoreScaleState
    {
MOVE_LIFT, DEPLOY_INTAKE, SPIT_OUT_CUBE, FINISHED
    }

// --------------------VARIABLES--------------------

// ================Override Related================



// ================FORKLIFT================
// Used in forkliftUpdate()
private ForkliftState liftState = ForkliftState.STAY_AT_POSITION;

// used to tell the forklift which direction it should be moving
private ForkliftDirectionState forkliftDirection = ForkliftDirectionState.NEUTRAL;

private double forkliftTargetHeight = 0.0;

private double forkliftTargetSpeed = 0.0;

private double currentMinLiftPosition = 0;
// ========================================

// ================INTAKE==================
// private boolean deployedArm = false;

// variable that controls the deploy intake state machine
private DeployState deployIntakeState = DeployState.NOT_DEPLOYED;

private pushOutState pushState = pushOutState.INIT;

private IntakeState intakeState = IntakeState.STOP;

private boolean isPullingIn = true;

private boolean lastIntakeButtonStatus = false;

private boolean intakeOverride = false;

private boolean isRunningPushOutCubeAuto = false;

// ========================================

private scoreScaleState scaleState = scoreScaleState.MOVE_LIFT;

// -------------------------------------------------

// --------------------CONSTANTS--------------------

// ================FORKLIFT================
private final double FORKLIFT_MAX_HEIGHT = 100;

private final double FORKLIFT_WITH_CUBE_MIN_HEIGHT = 5.0;

private final double FORKLIFT_NO_CUBE_MIN_HEIGHT = 1.0;

private final double FORKLIFT_DEFAULT_SPEED_UP = .3;

private final double FORKLIFT_DEFAULT_SPEED_DOWN = .3;

private final double FORKLIFT_STAY_UP_SPEED = 0.0; // -.15;

private final double FORKLIFT_STAY_UP_WITH_CUBE = .1;

private final double SCALE_HEIGHT = 80;
// =========================================

// ================INTAKE===================

private final double INTAKE_SPEED = .5;

private final double INTAKE_STOP_WITH_CUBE = .1;

// how many degrees the intake deploy motor needs to turn for the intake
// to be fully deployed
private final double INTAKE_DEPLOY_TICKS = 190;

// the encoder value that counts as the intake being retracted
private final double INTAKE_RETRACT_TICKS = 10.0;

private final double INTAKE_DEPLOY_SPEED = .3;

// speed we retract the intake mechanism at
private final double INTAKE_RETRACT_SPEED = -INTAKE_DEPLOY_SPEED;

private final double EJECT_TIME = 2.0;

// =========================================

private static final double JOYSTICK_DEADBAND = .2;

// ---------------------------------------------
}
