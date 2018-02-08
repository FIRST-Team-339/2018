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
 * @param operatorJoystick
 *            the joystick used when operting the forklift
 */
public void moveForkliftWithController (Joystick operatorJoystick)
{
    // If we move the forklift up with the joystick, only do so if we are below
    // the max height.
    if (operatorJoystick.getY() <= -JOYSTICK_DEADBAND
            && this.getForkliftHeight() <= FORKLIFT_MAX_HEIGHT)
        {
        this.forkliftSpeedUp = FORKLIFT_SPEED_UP;
        this.liftState = forkliftState.MOVING_UP;
        }
    // If we move the forklift down with the joystick, only do so if we are
    // above the min height
    else if (operatorJoystick.getY() >= JOYSTICK_DEADBAND
            && this.getForkliftHeight() >= FORKLIFT_MIN_HEIGHT)
        {
        this.forkliftSpeedDown = FORKLIFT_SPEED_DOWN;
        this.liftState = forkliftState.MOVING_DOWN;
        }
    // If we are not using the joysticks and the lift is near the bottom, then
    // the lift is in the starting position.
    else if (this.getForkliftHeight() <= FORKLIFT_MIN_HEIGHT
            + LIFT_TOLERANCE)
        {
        this.liftState = forkliftState.AT_STARTING_POSITION;
        }
    // If we are not moving the forklift, and it is above the starting position,
    // then stay there with a little voltage.
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

    // If the requested position is above the forklift height, then
    // the forklift is going up
    boolean mustGoUp = position > getForkliftHeight();

    this.forkliftHeightForMoveLiftDistance = position;

    // If we will travel up, then set it so for the state machine
    if (mustGoUp == true && this
            .getForkliftHeight() <= this.forkliftHeightForMoveLiftDistance
                    + LIFT_TOLERANCE)
        {
        this.forkliftSpeedUp = -forkliftSpeed;
        this.liftState = forkliftState.MOVING_UP;
        return this.finishedForkliftMove;
        }
    // If we will travel down, then set it so for the state machine
    else if (mustGoUp == false && this
            .getForkliftHeight() >= this.forkliftHeightForMoveLiftDistance
                    + LIFT_TOLERANCE)
        {
        this.forkliftSpeedDown = forkliftSpeed;
        this.liftState = forkliftState.MOVING_DOWN;
        }
    // We have finished moving the forklift! hurray!
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
    // If the requested position is greater than the current position, set the
    // state machine to go up.
    if (this.getForkliftHeight() < position)
        {
        setLiftPosition(position, Math.abs(FORKLIFT_SPEED_UP));
        }
    // Else, we are going down.
    else
        {
        setLiftPosition(position, Math.abs(FORKLIFT_SPEED_DOWN));
        }
    return this.finishedForkliftMove;
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
 * Only works if the intake mechanism is not currently already deployed
 * 
 * REQUIRED to be called before teleop periodic.
 * 
 * If the override boolean for this function is true, will call the override
 * code to always move the deployIntake down
 * 
 * @return true if the arm is down, false if it is still moving
 */
public boolean deployCubeIntake ()
{
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
 * @return true if the mechanism is retracted/ in the robot, false is otherwise
 *         (hasn't finished retracting)
 * 
 * @author Cole Ramos
 */
public boolean retractCubeIntake ()
{
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
 * Moves the intake based on the speed provided.
 * 
 * @param speed
 *            How fast the motors should move, in percent. (-1.0 to 1.0)
 *            Negative is pushing out, positive is pulling in
 * 
 */
public void moveIntake (double speed)
{
    // Deadband calcs
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
 * @param button
 *            Button/buttons used when intaking a cube
 * 
 * @return true if we are in control of a cube
 */
public boolean intakeCube (boolean button)
{
    // Button is pressed? tell the state machine we are intaking the cube
    if (button)
        {
        this.isRunningIntakeCube = true;
        // is the cube sensor triggered? No? then continue intaking.
        if (this.intakeSwitch.isOn() == false)
            {
            this.intakeMotor.set(INTAKE_SPEED);
            return false;
            }
        // Cube sensor is triggered? Yes? then stop the motors and return true.
        this.intakeMotor.set(0);
        return true;
        }
    // Button not pressed? we are not intaking cube. Return false.
    this.isRunningIntakeCube = false;
    return false;

}


/**
 * Intakes a cube, regardless of whether or not the cube sensor is triggered.
 * This is to make sure we are not useless if the sensor fails or falls
 * out of alignment.
 * 
 * @param button
 *            Button / buttons used when overriding the intake.
 */
public void intakeCubeOverride (boolean button)
{
    // If button is pressed, then run the intake motors.
    if (button)
        {
        this.isRunningIntakeCubeOverride = true;
        this.intakeMotor.set(INTAKE_SPEED);
        }
    // If not pressed, then don't.
    else
        {
        this.isRunningIntakeCubeOverride = false;
        }
}



/**
 * Ejects the cube using a button.
 * 
 * @param button
 *            a button on a joystick
 */
public void pushOutCubeTeleop (boolean button)
{
    // If button is pressed, then push out the cube.
    if (button)
        {
        this.isRunningPushOutCubeTeleop = true;
        this.intakeMotor.set(-INTAKE_SPEED);
        }
    // If not pressed, then don't.
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

/**
 * Autonomously scores a cube on the switch using the forklift, intake and
 * intake deploy
 * 
 * @return true if a cube has been scored in the switch
 */
public boolean scoreSwitch ()
{
    switch (switchState)
        {
        // If the intake has not been deployed already, then do so.
        case DEPLOY_INTAKE:
            if (deployCubeIntake() == true)
                {
                switchState = scoreSwitchState.MOVE_LIFT;
                }
            break;
        // Move the lift to the switch height, and move on when it's finished
        case MOVE_LIFT:
            if (setLiftPosition(SWITCH_HEIGHT,
                    FORKLIFT_SPEED_UP) == true)
                {
                switchState = scoreSwitchState.SPIT_OUT_CUBE;
                }
            break;
        // Eject the cube (onto the switch preferably)
        case SPIT_OUT_CUBE:
            if (pushOutCubeAuto() == true)
                {
                switchState = scoreSwitchState.FINISHED;
                }
            break;
        // If we have an undefined state input, for debugging
        default:
            System.out.println("Error finding state " + switchState
                    + " in CubeManipulator.scoreSwitch()");
            // We have finished (hopefully) scoring on the switch! Hurrah!
        case FINISHED:
            stopEverything();
            this.switchState = scoreSwitchState.MOVE_LIFT;
            return true;
        }
    // We have not yet finished scoring the switch
    return false;
}


// TODO make sure this works
/**
 * Scores a cube on a scale autonomously, using the forklift, intake and intake
 * deploy.
 * 
 * @return true if a cube has been scored in the scale
 */
public boolean scoreScale ()
{
    switch (scaleState)
        {
        // Make sure the intake is deployed before scoring on the switch
        case DEPLOY_INTAKE:
            System.out.println("Deploying intake");
            if (deployCubeIntake() == true)
                {
                System.out.println("Done deploying intake");
                switchState = scoreSwitchState.MOVE_LIFT;
                }
            break;
        // Move the lift to the height of the scale
        case MOVE_LIFT:
            System.out.println(
                    "Forklift height" + this.getForkliftHeight());

            if (setLiftPosition(SCALE_HEIGHT, .8) == true)
                {
                System.out.println(
                        "Finished raising lift: flork lyft height = "
                                + this.getForkliftHeight());
                switchState = scoreSwitchState.SPIT_OUT_CUBE;
                }
            else
                {
                setLiftPosition(SCALE_HEIGHT, .8);
                }
            break;
        // Push the cube on the scale
        case SPIT_OUT_CUBE:
            System.out.println("Spitting out cube");
            if (pushOutCubeAuto() == true)
                {
                scaleState = scoreScaleState.FINISHED;
                }
            break;
        // Debugging purposes: if we input a state that doesn't exist.
        default:
            System.out.println("Error finding state " + scaleState
                    + " in CubeManipulator.scoreScale()");
        case FINISHED:
            // We have finished scoring a cube on the scale (hopefully!)
            stopEverything();
            return true;
        }
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
    intakePushOutCubeUpdate();
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
        // Moves the forklift up
        case MOVING_UP:
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
        // Moves the forklift down
        case MOVING_DOWN:
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
        // Make the cube "hover" by sending a constant small voltage to the
        // forklift motor.
        case STAY_AT_POSITION:
            this.forkliftMotor.set(FORKLIFT_STAY_UP_SPEED);
            this.finishedForkliftMove = true;
            break;

        // Send no voltage to the motors, if all else fails (AND IT WILL, I SAY
        // YOU!)
        default:
        case AT_STARTING_POSITION:
            this.forkliftMotor.set(FORKLIFT_AT_STARTING_POSITION);
            break;
        }
}


/**
 * Update method for the deployIntake functions. Allows the deployIntake
 * code to use their state machine. deployIntake and related functions
 * will not work unless this updateFunction is called
 * 
 * @author Cole Ramos
 * 
 */
public void deployIntakeUpdate ()
{

    System.out.println("deployIntakeState: " + deployIntakeState);
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
            if (this.intakeDeployEncoder
                    .get() >= INTAKE_DEPLOY_ANGLE
                            - INTAKE_DEPLOY_COMPENSATION)
                {

                // stops the intake deploy motor if we've turned far enough;
                // FINISHED does this as well, but doing it here helps
                // keep the motor from overshooting too much
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
            this.intakeDeployMotor.set(-INTAKE_DEPLOY_SPEED);
            if (this.intakeDeployEncoder
                    .get() <= INTAKE_RETRACT_ANGLE)
                {
                // brings back in the intake mechanism until the intake
                // deploy
                // encoder reads the INTAKE_RETRACT_ANGLE (usually 0)
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.NOT_DEPLOYED;
                }
            break;

        case OVERRIDE_DEPLOY: // override that deploys the intake mechanism
                              // regardless of encoders
            this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
            break;

        case OVERRIDE_RETRACT: // override that retracts the intake mechanism
                               // regardless of encoders
            this.intakeDeployMotor.set(-INTAKE_DEPLOY_SPEED);
            break;

        // we shouldn't ever get here, but in case we do, stop the intake
        // deploy motor
        default:
            this.intakeDeployMotor.set(0.0);
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
 */
public void intakePushOutCubeUpdate ()
{
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
NOT_DEPLOYED, DEPLOYING, DEPLOYED, RETRACTING, OVERRIDE_DEPLOY, OVERRIDE_RETRACT
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

// ================Override Related================



// ================FORKLIFT================
// Used in forkliftUpdate()
private forkliftState liftState = forkliftState.AT_STARTING_POSITION;

private double forkliftHeightForMoveLiftDistance = 0;

private boolean finishedForkliftMove = false;

private double forkliftSpeedUp = 0;

private double forkliftSpeedDown = 0;
// ========================================

// ================INTAKE==================
// private boolean deployedArm = false;

// variable that controls the deploy intake state machine
private DeployState deployIntakeState = DeployState.NOT_DEPLOYED;

private pushOutState pushState = pushOutState.INIT;

// determines whether or not the intake motor should be stopped
private boolean stopIntake = false;

// the following booleans are for determining functions are using the intake

private boolean isRunningIntakeCube = false;

private boolean isRunningIntakeCubeOverride = false;

private boolean isRunningPushOutCubeAuto = false;

private boolean isRunningPushOutCubeTeleop = false;
// ========================================

private scoreScaleState scaleState = scoreScaleState.MOVE_LIFT;

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

private final double SCALE_HEIGHT = 72;
// =========================================

// ================INTAKE===================
private final double INTAKE_SPEED = .5;

// how many degrees the intake deploy motor needs to turn for the intake
// to be fully deployed
private final double INTAKE_DEPLOY_ANGLE = 75;

// the encoder value that counts as the intake being retracted
private final double INTAKE_RETRACT_ANGLE = 0.0;

// constant subtracted from the INTAKE_DEPLOY_ANGLE to help keep us
// from overshooting; needs to be tuned on the new robot
private final double INTAKE_DEPLOY_COMPENSATION = 0.0;

private final double INTAKE_DEPLOY_SPEED = .3;

private final double EJECT_TIME = 2.0;
// =========================================

private final double JOYSTICK_DEADBAND = .2;

// ---------------------------------------------
}
