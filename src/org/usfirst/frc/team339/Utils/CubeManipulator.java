package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
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
 * @param operatorJoystick
 *            the joystick used when operting the forklift
 * 
 * @author C.R.
 */
public void moveForkliftWithController (Joystick operatorJoystick)
{
    // If we move the forklift up with the joystick, only do so if we are below
    // the max height.
    if (operatorJoystick.getY() <= -JOYSTICK_DEADBAND
            && this.getForkliftHeight() <= FORKLIFT_MAX_HEIGHT)
        {
        // this.liftState = ForkliftState.MOVING_UP_MAX;
        this.setLiftPosition(FORKLIFT_MAX_HEIGHT, .5);
        }
    // If we move the forklift down with the joystick, only do so if we are
    // above the min height; we use the no cube version because it is lower, and
    // if we have a cube the setLiftPosition code will automatically use the
    // with cube mininimum height anyway
    else if (operatorJoystick.getY() >= JOYSTICK_DEADBAND
            && this.getForkliftHeight() >= FORKLIFT_MIN_HEIGHT_NO_CUBE)
        {
        // this.liftState = ForkliftState.MOVING_DOWN_MIN;
        this.setLiftPosition(FORKLIFT_MIN_HEIGHT_NO_CUBE, .5);
        }
    // If we are not moving the forklift, and it is above the starting position,
    // then stay there with a little voltage.
    else
        {
        this.liftState = ForkliftState.STAY_AT_POSITION;
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
    // this.finishedForkliftMove = false;
    //
    // // If the requested position is above the forklift height, then
    // // the forklift is going up
    // boolean mustGoUp = position > getForkliftHeight();
    //
    // this.forkliftTargetHeight = position;
    //
    // // If we will travel up, then set it so for the state machine
    // if (mustGoUp == true && this
    // .getForkliftHeight() <= this.forkliftTargetHeight
    // + LIFT_TOLERANCE)
    // {
    // // System.out.println("Decided to move up");
    // this.forkliftCurrentSpeedUp = -forkliftSpeed;
    // this.liftState = ForkliftState.MOVING_UP_MAX;
    // return this.finishedForkliftMove;
    // }
    // // If we will travel down, then set it so for the state machine
    // else if (mustGoUp == false && this
    // .getForkliftHeight() >= this.forkliftTargetHeight
    // + LIFT_TOLERANCE)
    // {
    // // System.out.println("Decided to move down");
    // this.forkliftCurrentSpeedDown = forkliftSpeed;
    // this.liftState = ForkliftState.MOVING_DOWN_MIN;
    // }
    // // We have finished moving the forklift! hurray!
    // else
    // {
    // this.finishedForkliftMove = true;
    // liftState = ForkliftState.STAY_AT_POSITION;
    // }
    // return this.finishedForkliftMove;

    // if the forklift is below the target position, set state to
    // MOVING_TO_POSITION


    forkliftTargetHeight = position;
    forkliftTargetSpeed = Math.abs(forkliftSpeed);


    if (this.getForkliftHeight() < forkliftTargetHeight
            - LIFT_UNDERSHOOT_TOLERANCE)
        {
        liftState = ForkliftState.MOVING_TO_POSITION;
        }
    // if the forklift is above the target position, set state to
    // MOVING_TO_POSITION
    else if (this.getForkliftHeight() > forkliftTargetHeight
            + LIFT_OVERSHOOT_TOLERANCE)
        {
        liftState = ForkliftState.MOVING_TO_POSITION;
        }
    // if the forklift is close enough to the target position, then set state to
    // STAY_AT_POSITION
    else
        {
        liftState = ForkliftState.STAY_AT_POSITION;
        }


    // return true is we are done moving, false is we are still going
    return liftState == ForkliftState.STAY_AT_POSITION;


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
        setLiftPosition(position, Math.abs(FORKLIFT_DEFAULT_SPEED_UP));
        }
    // Else, we are going down.
    else
        {
        setLiftPosition(position,
                Math.abs(FORKLIFT_DEFAULT_SPEED_DOWN));
        }
    // TODO CHANGE WHAT THIS IS RETURNING!!!!!
    return true;
}

// ========================INTAKE FUNCTIONS========================


public double getIntakeAngle ()
{
    return this.intakeDeployEncoder.get();
}

/**
 * Tell the intake motor to stop by setting it to zero. Should be changed to
 * private because NO ONE SHOULD BE USING THIS OUTSIDE OF THE STATE MACHINE
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
 * Method that handles calling the deployCubeIntake and retractCubeIntake
 * functions, as well using the overrides for deploy and retract
 * 
 * @param deployButton
 *            Joystick button used to deploy the intake mechanism
 * @param retractButton
 *            Joystick button used to retract the intake mechanism
 * @param overrideButton
 *            Joystick button used to toggle the overrides; hold this down and
 *            hit the other buttons in order to use the overriden version of the
 *            functions
 * 
 * @return true if the intake (state) is deployed, else if otherwise
 * 
 * @author C.R.
 */
public boolean deployRetractIntakeByButtons (boolean deployButton,
        boolean retractButton,
        boolean overrideButton)
{

    if (overrideButton == false) // run the non-overriden deploy and retract
                                 // functions
        {
        if (deployButton == true) // if the deployButton is being pressed
            {
            // ask the intake mechanism to deploy if it hasn't been deployed
            // already
            deployCubeIntake();
            }

        if (retractButton == true) // if the reractButton is being pressed
            {
            // ask the intake mechanism to retract if it is already deployed
            retractCubeIntake();
            }
        }
    else // if overrideButton == true, run the overidden versions of the deploy
         // and retract functions
        {
        if (deployButton == true) // if the deploy button is being pressed
            {
            // set the deployIntakeState to OVERRIDE_DEPLOY so the deployRetract
            // state machine knows to deploy no matter what the encoders say
            deployIntakeState = DeployState.OVERRIDE_DEPLOY;
            this.lastOverride = DeployState.OVERRIDE_DEPLOY;
            }
        // if the deploy button is not being pressed and the retract button is
        // being pressed
        else if (retractButton == true)
            {
            // set the deployIntakeState to OVERRIDE_RETRACT so the
            // deployRetract state machine knwos to retract no matter what the
            // encoders say
            deployIntakeState = DeployState.OVERRIDE_RETRACT;
            this.lastOverride = DeployState.OVERRIDE_RETRACT;
            }
        else
            {
            // default state when we are in override (if we aren't trying to
            // deploy or retract); stops the intake motor
            deployIntakeState = DeployState.OVERRIDE_DEFAULT;
            }
        }

    // if we were using override last call but are not now
    if (usedOverrideLast == true && overrideButton == false)
        {
        // sets the deployIntakeState back to its appropriate non override state
        deployIntakeState = DeployState.OVERRIDE_END;
        }


    usedOverrideLast = overrideButton;
    // return true is we are not in an override state and the intake mechanism
    // is deployed
    return deployIntakeState == DeployState.DEPLOYED;
}

// boolean used to tell if we were in override last call; used so the
// deployRetractIntakeByButtons
private boolean usedOverrideLast = false;


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
            // System.out.println("Error finding state " + pushState
            // + " in CubeManipulator.pushOutCubeAuto()");
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
 * Not currently working
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
            // System.out.println("Moving lift");
            if (setLiftPosition(SWITCH_HEIGHT,
                    FORKLIFT_DEFAULT_SPEED_UP) == true)
                {
                switchState = scoreSwitchState.SPIT_OUT_CUBE;
                }
            break;
        // Eject the cube (onto the switch preferably)
        case SPIT_OUT_CUBE:
            // System.out.println("Spitting out cube");
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
            if (deployCubeIntake() == true)
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
    SmartDashboard.putString("Forklift State:", liftState + "");
    SmartDashboard.putString("Forklift Direction State",
            "" + forkliftDirection);
    SmartDashboard.putString("Target Height ",
            forkliftTargetHeight + "");


    double SLOW_LIFT_SPEED = .3;

    // main switch statement for the forklift state machine
    switch (liftState)
        {
        // Moves the forklift up


        // case MOVING_UP_MAX:
        // if (// Math.abs(
        // this.getForkliftHeight() >= FORKLIFT_MAX_HEIGHT)
        // {
        // this.liftState = ForkliftState.STAY_AT_POSITION;
        // }
        // else
        // {
        // this.forkliftMotor.set(this.forkliftCurrentSpeedUp);
        // }
        // break;

        // Moves the forklift down
        // case MOVING_DOWN_MIN:
        // if (this.getForkliftHeight() <= FORKLIFT_MIN_HEIGHT_NO_CUBE
        // + LIFT_TOLERANCE)
        // {
        // this.liftState = ForkliftState.STAY_AT_POSITION;
        // }
        // else
        // {
        // this.forkliftMotor.set(this.forkliftCurrentSpeedDown);
        // }
        // break;

        case MOVING_TO_POSITION:
            System.out.println("moving to position");
            // two if statements to prevent forkliftTargetHeight from being past
            // the min and max heights

            if (forkliftTargetHeight > FORKLIFT_MAX_HEIGHT)
                {
                // set the target height to the max height if the target height
                // was too high
                forkliftTargetHeight = FORKLIFT_MAX_HEIGHT;
                }
            if (this.hasCube() == true
                    && forkliftTargetHeight < FORKLIFT_MIN_HEIGHT_WITH_CUBE)
                {
                // set the target height to the min height (with cube version)
                // if the target height was too low
                forkliftTargetHeight = FORKLIFT_MIN_HEIGHT_WITH_CUBE;
                }

            if (this.hasCube() == false
                    && forkliftTargetHeight < FORKLIFT_MIN_HEIGHT_NO_CUBE)
                {
                // set the target height to the min height (no cube version) if
                // the target height was too low
                forkliftTargetHeight = FORKLIFT_MIN_HEIGHT_NO_CUBE;
                }

            // determine whether or not the forklift needs to go up or down, and
            // set the forklift speed variables as appropriate
            if (this.getForkliftHeight() < forkliftTargetHeight
                    - LIFT_UNDERSHOOT_TOLERANCE)
                {
                forkliftCurrentSpeedUp = forkliftTargetSpeed;
                forkliftDirection = ForkliftDirectionState.MOVING_UP;
                }
            else if (this.getForkliftHeight() > forkliftTargetHeight
                    + LIFT_OVERSHOOT_TOLERANCE)
                {

                forkliftCurrentSpeedDown = -forkliftTargetSpeed;
                forkliftDirection = ForkliftDirectionState.MOVING_DOWN;
                }
            else
                {
                System.out.println("at position");
                forkliftDirection = ForkliftDirectionState.AT_POSITION;
                }

            // switch statement for moving the forkliftMotor properly based off
            // which direction we want it to go
            switch (forkliftDirection)
                {

                case MOVING_UP:

                    // if (this.getForkliftHeight() > forkliftTargetHeight
                    // - LIFT_UNDERSHOOT_TOLERANCE
                    // - SLOW_DOWN_DISTANCE)
                    // this.forkliftMotor.set(SLOW_LIFT_SPEED);
                    // else

                    this.forkliftMotor.set(forkliftCurrentSpeedUp);
                    break;
                case MOVING_DOWN:

                    // if (this.getForkliftHeight() < forkliftTargetHeight
                    // + LIFT_OVERSHOOT_TOLERANCE
                    // + SLOW_DOWN_DISTANCE)
                    // this.forkliftMotor.set(SLOW_LIFT_SPEED);
                    // else
                    this.forkliftMotor
                            .set(forkliftCurrentSpeedDown);

                    break;
                default: // if something goes wrong, print it out to console
                         // that we got to default and fall through to
                         // AT_POSITION's case
                    System.out.println(
                            "Reached default in the forkliftDirection switch"
                                    + " in forkliftUpdate in CubeManipulator;");
                case AT_POSITION:
                    // gives the forklift motor the appropriate voltage for it
                    // to hold its position; which voltage we use is dependent
                    // on whether or not we have a cube
                    if (this.hasCube() == true)
                        {
                        this.forkliftMotor.set(FORKLIFT_STAY_UP_SPEED);
                        }
                    else
                        {
                        this.forkliftMotor
                                .set(FORKLIFT_STAY_UP_WITH_CUBE);
                        }
                    liftState = ForkliftState.STAY_AT_POSITION;
                    break;
                } // end the forkliftDirection switch statement

            break;

        // Make the cube "hover" by sending a constant small voltage to the
        // forklift motor.
        default:
            // print out we reached the default case (which we shouldn't have),
            // then fall through to STAY_AT_POSITION
            System.out.println(
                    "Reached default in the liftState switch in "
                            + "forkliftUpdate in CubeManipulator");
        case STAY_AT_POSITION:
            // gives the forklift motor the appropriate voltage for it
            // to hold its position; which voltage we use is dependent
            // on whether or not we have a cube

            // if the forklift doesn't have cube and is close enough to the
            // minimum height, assume the operator wants the forklift to go back
            // so zero, so "drift" back to the floor
            if (this.hasCube() == false && this
                    .getForkliftHeight() < FORKLIFT_MIN_HEIGHT_NO_CUBE
                            + LIFT_OVERSHOOT_TOLERANCE)
                {
                SmartDashboard.putString("FORKLIFT SPEED (NOT MOTOR)",
                        "drift");
                this.forkliftMotor.set(FORKLIFT_DRIFT_DOWN_SPEED);
                }
            else if (this.hasCube() == true)
                {
                SmartDashboard.putString("FORKLIFT SPEED (NOT MOTOR)",
                        "stay with cube");
                this.forkliftMotor
                        .set(FORKLIFT_STAY_UP_WITH_CUBE);
                }
            else
                {
                SmartDashboard.putString("FORKLIFT SPEED (NOT MOTOR)",
                        "stay up no cube");
                this.forkliftMotor.set(FORKLIFT_STAY_UP_SPEED);
                }
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

            if (this.getIntakeAngle() >= INTAKE_DEPLOY_ANGLE
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
            this.intakeDeployMotor.set(INTAKE_RETRACT_SPEED);
            if (this.intakeDeployEncoder
                    .get() <= INTAKE_RETRACT_ANGLE
                            - INTAKE_RETRACT_COMPENSATION)
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
            this.intakeDeployMotor.set(INTAKE_RETRACT_SPEED);
            break;
        case OVERRIDE_DEFAULT: // default state if the override button is being
                               // held; just tells the motor to stop (but might
                               // keep moving anyway because of gravity)
            this.intakeDeployMotor.set(0.0);
            break;
        // sets the deployIntakeState back to its proper non override state
        // based on encoder values
        case OVERRIDE_END:
            // if the encoder says we are deployed, stop the motor and set state
            // to DEPLOYED
            if (this.getIntakeAngle() >= INTAKE_DEPLOY_ANGLE
                    - INTAKE_DEPLOY_COMPENSATION)
                {
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.DEPLOYED;
                }
            // if the encoder says the intake is still in the robot, stop the
            // motor and set the state to NOT_DEPLOYED
            else if (this.intakeDeployEncoder
                    .get() <= INTAKE_RETRACT_ANGLE
                            - INTAKE_RETRACT_COMPENSATION)
                {
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.NOT_DEPLOYED;
                }
            // if the encoder isn't telling us we are deployed and retracted,
            // and the time we thing we did in override was retract, tell the
            // intake to keep retracting
            else if (this.lastOverride == DeployState.OVERRIDE_RETRACT)
                {
                deployIntakeState = DeployState.RETRACTING;
                }
            // otherwise, tell the intake to deploy
            else
                {
                deployIntakeState = DeployState.DEPLOYING;
                }

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
private static enum ForkliftState
    {
MOVING_TO_POSITION, STAY_AT_POSITION
    }

/**
 * States used inside the MOVNG_TO_POSITION state (from ForkliftState) which way
 * the forklift is
 * moving
 * 
 * @author C.R.
 *
 */
private static enum ForkliftDirectionState
    {
MOVING_UP, MOVING_DOWN, AT_POSITION
    }

/**
 * Enum used for controlling the deploy intake state machine
 * 
 * @author Cole Ramos
 */
private static enum DeployState
    {
NOT_DEPLOYED, DEPLOYING, DEPLOYED, RETRACTING, OVERRIDE_DEPLOY, OVERRIDE_RETRACT, OVERRIDE_DEFAULT, OVERRIDE_END
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
private ForkliftState liftState = ForkliftState.STAY_AT_POSITION;

// used to tell the forklift which direction it should be moving
private ForkliftDirectionState forkliftDirection = ForkliftDirectionState.AT_POSITION;

private double forkliftTargetHeight = 0.0;

private double forkliftTargetSpeed = 0.0;

// TODO remove and replace with forkliftTargetSpeed
private double forkliftCurrentSpeedUp = 0;

// TODO remove and replace with forkliftTargetSpeed
private double forkliftCurrentSpeedDown = 0;
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

private final double FORKLIFT_MIN_HEIGHT_NO_CUBE = 2.0;

private final double FORKLIFT_MIN_HEIGHT_WITH_CUBE = 4.0;

private final double FORKLIFT_DEFAULT_SPEED_UP = .9;

private final double FORKLIFT_DEFAULT_SPEED_DOWN = .4;

private final double FORKLIFT_STAY_UP_SPEED = 0.0; // -.15;

private final double FORKLIFT_STAY_UP_WITH_CUBE = 0.0;

// used to let the forklift drift the final couple inches whenever we are moving
// it back to the bottom/ starting position
private final double FORKLIFT_DRIFT_DOWN_SPEED = 0.0;

// TODO remove
private final double LIFT_TOLERANCE = 3;


// how far the lift is allowed to be over the target height
private final double LIFT_OVERSHOOT_TOLERANCE = 5.0;

// how far the lift is allowed to be under the target height
private final double LIFT_UNDERSHOOT_TOLERANCE = 1.0;

// how close the forklift is to te target height before it starts slowing down
private final double SLOW_DOWN_DISTANCE = 5.0;

private final double SWITCH_HEIGHT = 30;

private final double SCALE_HEIGHT = 80;
// =========================================

// ================INTAKE===================


private final double INTAKE_SPEED = .5;

// how many degrees the intake deploy motor needs to turn for the intake
// to be fully deployed
private final double INTAKE_DEPLOY_ANGLE = 75;

// the encoder value that counts as the intake being retracted
private final double INTAKE_RETRACT_ANGLE = 10.0;

// constant subtracted from the INTAKE_DEPLOY_ANGLE to help keep us
// from overshooting; needs to be tuned on the new robot
private final double INTAKE_DEPLOY_COMPENSATION = 20.0;

// constant SUBTRACTED from the INTAKE_RETRACT_ANGLE to help keep us from over
// or under shooting; needs to be tuned on the new robot; if you want the intake
// motor to be stopping earlier, should be negative
private final double INTAKE_RETRACT_COMPENSATION = -10.0;

private final double INTAKE_DEPLOY_SPEED = .2;

// speed we retract the intake mechanism at
private final double INTAKE_RETRACT_SPEED = -INTAKE_DEPLOY_SPEED;

private final double EJECT_TIME = 2.0;

// is set elsewhere in the code to OVERRIDE_DEPLOY or OVERRIDE_RETRACT,
// depending on which one was used last; used in the OVERRIDE_END state in the
// deploy/retract intake state machine
private DeployState lastOverride = DeployState.OVERRIDE_DEPLOY;
// =========================================

public static final double JOYSTICK_DEADBAND = .2;

// ---------------------------------------------
}
