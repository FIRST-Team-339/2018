package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.DoubleSolenoid;
import org.usfirst.frc.team339.HardwareInterfaces.KilroyEncoder;
import org.usfirst.frc.team339.HardwareInterfaces.LightSensor;
import org.usfirst.frc.team339.HardwareInterfaces.MomentarySwitch;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class for forklift and intake subsystems which includes a Forklift,
 * intake motors, and an intake deploy mechanism.
 * 
 * @author Becky Button
 * @param <Button>
 *
 */
public class CubeManipulator<Button>
{
// ---------------HARDWARE---------------------

// ================FORKLIFT================
private SpeedController forkliftMotor = null;

private KilroyEncoder forkliftEncoder = null;

private LightSensor armIR = null;

private boolean newCode = true;

// true if we want to use the code to stop the forklift if the armIR
// thinks we are about to hit the scale
// TODO fix IR
private boolean usingArmIRStop = false;


private MomentarySwitch climbButton = null;
// ========================================

// ================INTAKE==================
private SpeedController intakeMotor = null;

private SpeedController intakeDeployMotor = null;

private static KilroyEncoder intakeDeployEncoder = null;

private LightSensor intakeSwitch = null;

private Timer switchTimer = null;

private Servo deployFoldingServo = null;

private DoubleSolenoid armIntakeSolenoid = null;
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
 * @param deployFoldingServo
 *            Servo used to fold the deploy down in order to climb.
 */
public CubeManipulator (SpeedController forkliftMotor,
        SpeedController intakeMotor, LightSensor intakeSwitch,
        KilroyEncoder forkliftEncoder, SpeedController intakeDeploy,
        KilroyEncoder intakeDeployEncoder, Timer timer,
        Servo deployFoldingServo)
{
    this.forkliftMotor = forkliftMotor;
    this.intakeMotor = intakeMotor;
    this.intakeSwitch = intakeSwitch;
    this.forkliftEncoder = forkliftEncoder;
    this.intakeDeployMotor = intakeDeploy;
    CubeManipulator.intakeDeployEncoder = intakeDeployEncoder;
    this.switchTimer = timer;
    this.deployFoldingServo = deployFoldingServo;

    this.currentForkliftMaxHeight = FORKLIFT_MAX_HEIGHT;

}

/**
 * Creates the CubeManipulator class, which conrols the movement of the
 * forklift, intake, and climbing for autonomous and teleop functions
 * 
 * BONUS OF NEW SOLENOID
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
 * @param deployFoldingServo
 *            Servo used to fold the deploy down in order to climb.
 * @param armIntakeSolenoid
 *            Solenoid that controls whether the intake arms are open or closed
 */
public CubeManipulator (SpeedController forkliftMotor,
        SpeedController intakeMotor, LightSensor intakeSwitch,
        KilroyEncoder forkliftEncoder, SpeedController intakeDeploy,
        KilroyEncoder intakeDeployEncoder, Timer timer,
        Servo deployFoldingServo, DoubleSolenoid armIntakeSolenoid,
        MomentarySwitch climbButton)
{
    this.forkliftMotor = forkliftMotor;
    this.intakeMotor = intakeMotor;
    this.intakeSwitch = intakeSwitch;
    this.forkliftEncoder = forkliftEncoder;
    this.intakeDeployMotor = intakeDeploy;
    CubeManipulator.intakeDeployEncoder = intakeDeployEncoder;
    this.switchTimer = timer;
    this.deployFoldingServo = deployFoldingServo;
    this.armIntakeSolenoid = armIntakeSolenoid;
    Hardware.liftMotorTwo.follow(Hardware.liftMotorOne);
    this.climbButton = climbButton;

    this.currentForkliftMaxHeight = FORKLIFT_MAX_HEIGHT;

}

// CONSTRUCTOR TO INCLUDE THE ARMIR and the new code boolean
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
 * @param deployFoldingServo
 *            Servo used to fold the deploy down in order to climb.
 * @param armIRSensor
 *            IR used to see if were about to hit the scale
 * @param boolean
 *            boolean used to determine if we want to use the new code
 */
public CubeManipulator (SpeedController forkliftMotor,
        SpeedController intakeMotor, LightSensor intakeSwitch,
        KilroyEncoder forkliftEncoder, SpeedController intakeDeploy,
        KilroyEncoder intakeDeployEncoder, Timer timer,
        Servo deployFoldingServo, LightSensor armIRSensor)
{
    this.forkliftMotor = forkliftMotor;
    this.intakeMotor = intakeMotor;
    this.intakeSwitch = intakeSwitch;
    this.forkliftEncoder = forkliftEncoder;
    this.intakeDeployMotor = intakeDeploy;
    CubeManipulator.intakeDeployEncoder = intakeDeployEncoder;
    this.switchTimer = timer;
    this.deployFoldingServo = deployFoldingServo;
    this.armIR = armIRSensor;

    this.currentForkliftMaxHeight = FORKLIFT_MAX_HEIGHT;
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
        boolean overrideButton, boolean isClimbing)
{
    if (isClimbing == true)
        {
        this.currentForkliftDownSpeed = FORKLIFT_CLIMB_SCALAR;
        }
    else
        {
        this.currentForkliftDownSpeed = FORKLIFT_DOWN_JOYSTICK_SCALAR;
        }

    // Override button, ignore encoder.
    if (overrideButton == true)
        {
        this.forkliftTargetSpeed = speed;
        this.liftState = ForkliftState.MOVE_JOY;
        }
    else
        {
        // If we are past the max height or below the min, or we are using the
        // armIR and it is telling us to stop don't move the motors.
        if ((speed > 0
                && forkliftEncoder
                        .getDistance() > currentForkliftMaxHeight)
                || (speed < 0 && forkliftEncoder
                        .getDistance() < currentMinLiftPosition)
                || Math.abs(speed) < JOYSTICK_DEADBAND
                || (this.usingArmIRStop == true && this.armIR.isOn()
                        && this.forkliftEncoder
                                .getDistance() > USE_ARM_IR_HEIGHT
                        && speed > 0
                        && deployIntakeState == DeployState.DEPLOYED))

            {
            forkliftTargetSpeed = FORKLIFT_STAY_UP_SPEED; // @ANE take out

            return;
            }
        // Move the forklift the desired speed
        if (speed > 0)
            forkliftTargetSpeed = speed * FORKLIFT_UP_JOYSTICK_SCALAR;
        else
            forkliftTargetSpeed = speed * currentForkliftDownSpeed;

        // checks to see if we want to use new code

        // if (usingArmIRStop == true)
        // { // checks to see if the IR reads true, if so sets the state to
        // // stay at position because we're about to hit the stupid scale
        // if (armIR.isOn() == true && speed > 0 &&
        // deployIntakeState != DeployState.FOLDED
        // && deployIntakeState != DeployState.FOLD_ARM_DOWN
        // && deployIntakeState != DeployState.UNFOLD_ARM_UP)
        // {
        // this.liftState = ForkliftState.STAY_AT_POSITION;
        // } // if the override is off and the IR reads false then move
        // // based on joystick
        // else
        // {
        // this.liftState = ForkliftState.MOVE_JOY;
        // }
        //
        // }
        // else
        // {
        this.liftState = ForkliftState.MOVE_JOY;
        // }
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
    // Sets the target position and speed, enables "moving-to-position"
    // state.
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
    // If the requested position is greater than the current position, set
    // the
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

/**
 * Sets the maximum height for the lift. Use only for demo mode.
 * 
 * @param inches
 *            Maximum height, in inches.
 */
public void setMaxLiftHeight (int inches)
{
    this.currentForkliftMaxHeight = inches;
}

// ========================INTAKE FUNCTIONS========================

/**
 * Get the angle (in ticks) that the intakeDeployEncoder is reading
 * 
 * @return the ticks of the intakeDeployEncoder
 */
public static double getIntakeAngle ()
{
    return CubeManipulator.intakeDeployEncoder.get();
}

/**
 * Gets the speed the motor is being SET TO (in the code, not what is physically
 * happening on the robot)
 * 
 * @return the value being sent to motor
 */
public double getIntakeMotorSpeed ()
{
    return this.intakeMotor.get();
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
    if (override == true)
        {
        deployIntakeState = DeployState.OVERRIDE_DEPLOY;
        return true;
        }
    // advances the deploy intake state machine if it hasn't already been
    // deployed/ is deploying
    if (deployIntakeState == DeployState.NOT_DEPLOYED
            || deployIntakeState == DeployState.STOPPED
            || deployIntakeState == DeployState.OVERRIDE_DEPLOY
            || deployIntakeState == DeployState.UNFOLD_ARM_UP)

        {
        // Hardware.deployTimer.reset();
        // Hardware.deployTimer.start();
        deployIntakeState = DeployState.PRE_DEPLOYING;
        }

    if (newCode == false)
        {
        // returns whether or not the intake has finished deploying
        if (deployIntakeState == DeployState.DEPLOYED)
            {
            return true;
            }
        }
    else
        {
        if (deployIntakeState == DeployState.DEPLOYED
                && climbButton.isOnCheckNow() == true)
            {
            deployIntakeState = DeployState.FOLD_ARM_DOWN;
            }
        else
            {
            return true;
            }
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
 * Angles the deploy to the 45 degree angle while the button is held, and holds
 * it at that position for placing on the scale.
 */
public static boolean angleDeployForScale ()
{
    // System.out.println("TOP angleDeployForScale");
    if (deployIntakeState != DeployState.STOPPED
            && deployIntakeState != DeployState.OVERRIDE_DEPLOY
            && deployIntakeState != DeployState.OVERRIDE_RETRACT
    // deployIntakeState == DeployState.DEPLOYED
    // deployIntakeState != DeployState.STOPPED
    // && deployIntakeState != DeployState.OVERRIDE_DEPLOY
    // && deployIntakeState != DeployState.OVERRIDE_RETRACT
    // && deployIntakeState != DeployState.HOLDING_45
    // && deployIntakeState != DeployState.DEPLOY_45
    // && deployIntakeState != DeployState.POSITION_45
    // && deployIntakeState != DeployState.DEPLOYING
    // && deployIntakeState != DeployState.FOLD_ARM_DOWN
    // && deployIntakeState != DeployState.FOLDED
    // && deployIntakeState != DeployState.NOT_DEPLOYED
    // && deployIntakeState != DeployState.RETRACTING
    // && deployIntakeState != DeployState.UNFOLD_ARM_UP
    )
        {
        System.out.println("IN angleDeployForScale");
        deployIntakeState = DeployState.POSITION_45;
        // Hardware.deployTimer.reset();
        // Hardware.deployTimer.start();
        }

    return getIntakeAngle() < DEPLOY_45_POSITION_TICKS;// (deployIntakeState ==
                                                       // DeployState.HOLDING_45);
}

/**
 * Folds the deploy down to be able to climb well
 */
public void setDeployForClimb ()
{
    if (deployIntakeState == DeployState.DEPLOYED)
        {
        deployIntakeState = DeployState.FOLD_ARM_DOWN;
        }
    else if (deployIntakeState == DeployState.FOLDED)
        {
        deployIntakeState = DeployState.UNFOLD_ARM_UP;
        }
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
public void runIntake (boolean button, boolean pullInOverride,
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

    // If it's the buttons first run, then figure out if we will be pulling
    // the
    // cube in or pushing it out
    if (button == true && lastIntakeButtonStatus == false)
        {
        isPullingIn = hasCube() == false;
        }
    // If it's not the buttons first run but the button is still being
    // pressed,
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
 * Intakes the cube on a single button and an override.
 * 
 * @param button
 *            If the button is pressed, run the intake in.
 * @param override
 *            Intakes the cube if the button is pressed, and the override is
 *            pressed, regardless of the photoswitch
 */
public void intakeCube (boolean button, boolean override)
{
    if ((button && override) || (button && !hasCube()))
        this.intakeState = IntakeState.PULL_IN;
}

/**
 * Ejects the cube based on a button.
 * 
 * @param eject
 *            If the button is pressed, then eject the cube at high speed
 * @param place
 *            If the button is pressed, then eject the cube at low speed,
 *            effectively dropping it.
 */
public void ejectCube (boolean eject, boolean place)
{
    if (eject || place)
        this.intakeState = IntakeState.PUSH_OUT;

    isLowEject = place;
}

private boolean isLowEject = false;

/**
 * Pushes out the cube for autonomous only, with the default speed.
 * 
 * @return
 *         whether or not we have finished pushing out the cube
 */
public boolean pushOutCubeAuto ()
{
    return this.pushOutCubeAuto(this.currentEjectSpeed);
}

/**
 * Pushes out the cube. For use in autonomous only
 * 
 * @param speed
 *            how fast the intake should run.
 * 
 * @return true if this function is complete, false if still going
 */
public boolean pushOutCubeAuto (double speed)
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
                this.intakeMotor.set(-Math.abs(speed));
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
    return false;// this.intakeSwitch.isOn();
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
    // SmartDashboard.putString("scoreScale states",
    // scaleState.toString());
    switch (scaleState)
        {

        // Move the lift to the scale height, and move on when it's finished
        case MOVE_LIFT:
            if (setLiftPosition(SCALE_HEIGHT,
                    FORKLIFT_DEFAULT_SPEED_UP) == true)
                {
                scaleState = scoreScaleState.SPIT_OUT_CUBE;
                }
            break;
        // Eject the cube (onto the scale preferably)
        case SPIT_OUT_CUBE:
            // System.out.println("Spitting out cube");
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

private double getRetractSpeed ()
{
    if (getIntakeAngle() > INTAKE_DEPLOY_TICKS || hasCube())
        return INTAKE_RETRACT_SPEED_HIGH;
    // else
    return INTAKE_RETRACT_SPEED_LOW;
}

public boolean timedRetract90 ()
{
    Hardware.deployTimer.reset();
    Hardware.deployTimer.start();
    Hardware.intakeDeployArm.set(INTAKE_RETRACT_SPEED_HIGH);
    if (Hardware.deployTimer.get() >= TIME_TO_RETRACT_90)
        {
        Hardware.intakeDeployArm.set(0.0);
        return true;
        }

    return false;
}


public boolean timedDeploy90 ()
{
    Hardware.intakeDeployArm.set(INTAKE_DEPLOY_SPEED);
    System.out.println("WE GOT HERE");
    System.out.println("DEPLOY TIMER = " + Hardware.deployTimer.get());
    if (Hardware.deployTimer.get() >= TIME_TO_DEPLOY_90)
        {
        Hardware.intakeDeployArm.set(0.0);
        return true;
        }

    return false;
}


public boolean timedRetract45 ()
{
    Hardware.intakeDeployArm.set(INTAKE_RETRACT_SPEED_LOW);
    if (Hardware.deployTimer.get() >= TIME_TO_RETRACT_45)
        {
        Hardware.intakeDeployArm.set(0.0);
        return true;
        }

    return false;
}

public static boolean timedDeploy45 ()
{
    Hardware.intakeDeployArm.set(INTAKE_DEPLOY_SPEED);
    deployIntakeState = DeployState.DEPLOY_45;
    if (Hardware.deployTimer.get() >= TIME_TO_DEPLOY_45)
        {
        deployIntakeState = DeployState.DEPLOYED;
        Hardware.intakeDeployArm.set(0.0);
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
    // SmartDashboard.putString("Forklift Update", liftState.toString());
    // SmartDashboard.putString("Deploy State", deployIntakeState + "");
    // SmartDashboard.putString("Intake State", intakeState + "");

    // update the forklift state machine
    forkliftUpdate();
    // update the deployIntake state machine
    deployIntakeUpdate();
    // update the intake/ PushOut cube state machines
    intakeUpdate();
}

/**
 * Demo update function that calls the update functions for forklift,
 * and intake/ push out cube, allowing them to properly
 * use their state machines
 * 
 * @author Ashley Espeland
 * 
 */
public void demoUpdate ()
{
    // SmartDashboard.putString("Forklift Update", liftState.toString());
    // SmartDashboard.putString("Intake State", intakeState + "");

    // update the forklift state machine
    forkliftUpdate();

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
    // Make sure the lift stays up to prevent bad things when folding the
    // deploy
    if (deployIntakeState == DeployState.FOLD_ARM_DOWN
            || deployIntakeState == DeployState.FOLDED
            || deployIntakeState == DeployState.UNFOLD_ARM_UP)
        this.currentMinLiftPosition = FORKLIFT_DEPLOY_FOLDED_MIN_HEIGHT;
    else
        this.currentMinLiftPosition = FORKLIFT_NO_CUBE_MIN_HEIGHT;

    // main switch statement for the forklift state machine
    switch (liftState)
        {
        case MOVING_TO_POSITION:
            // Make sure we don't move past the MAX or MIN position
            if ((this.forkliftTargetHeight > currentForkliftMaxHeight)
                    || (this.forkliftTargetHeight < currentMinLiftPosition))
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
            // TODO test scaleIR code
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
                // we have NOT passed the value , keep going up.

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
                // we have NOT passed the value , keep going down.

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
            // print out we reached the default case (which we shouldn't
            // have),
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
            break;
        }
}

private boolean deployHasCube = false;

private boolean deployWas45 = false;

private double currentIntakeAngle = 0;

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
    // System.out.println("deploy state = " + deployIntakeState);
    // state machine for deploying the intake
    switch (deployIntakeState)
        {

        // initial state of the intake deploy motor; just stays still
        // until the deployCubeIntake() function is called
        case NOT_DEPLOYED:
            this.intakeDeployMotor.set(0.0);
            break;


        case PRE_DEPLOYING:
            // code to retract arm before deploying
            this.intakeDeployMotor.set(INTAKE_RETRACT_SPEED_HIGH);
            System.out.println("WE ARE ACTUALLY TRYING TO GO BACK");
            if (CubeManipulator.getIntakeAngle() <= INTAKE_RETRACT_TICKS
                    - DEGREES_TO_RETRACT_TO_DEPLOY)
                {
                // stops the intake deploy motor if we've turned far enough;
                // FINISHED does this as well, but doing it here helps
                // keep the motor from overshooting too much
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.DEPLOYING;
                }
            break;

        // moves the intake until the encoder reads that the arm has
        // turned the specified angle, then stops the intake deploy motor
        // and
        // moves to the next state
        case DEPLOYING:

            this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
            // System.out.println("YESSSSSSSSSSSSS");
            if (CubeManipulator.getIntakeAngle() >= INTAKE_DEPLOY_TICKS
                    - DEGREES_TO_RETRACT_TO_DEPLOY)
                {
                // stops the intake deploy motor if we've turned far enough;
                // FINISHED does this as well, but doing it here helps
                // keep the motor from overshooting too much
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.DEPLOYED;
                }


            // if (this.timedDeploy90() == true)
            // {
            //
            // deployIntakeState = DeployState.DEPLOYED;
            // }

            break;

        // final state in the state machine; stops the intake deploy
        // motor
        case DEPLOYED:
            this.intakeDeployMotor.set(0.0);
            break;

        // brings the intake mechanism back into the robot, and sets the
        // state to NOT_DEPLOYED
        case RETRACTING:
            this.intakeDeployMotor.set(getRetractSpeed());
            if (CubeManipulator.intakeDeployEncoder
                    .get() <= INTAKE_RETRACT_TICKS)
                {
                // brings back in the intake mechanism until the intake
                // deploy
                // encoder reads the INTAKE_RETRACT_ANGLE (usually 0)
                this.intakeDeployMotor.set(0.0);
                deployIntakeState = DeployState.NOT_DEPLOYED;
                }
            // if (this.timedRetract90() == true)
            // {
            // deployIntakeState = DeployState.NOT_DEPLOYED;
            // }

            break;

        case OVERRIDE_DEPLOY:
            this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
            // If the override is let go, then stop the deploy
            deployIntakeState = DeployState.STOPPED;
            break;
        case OVERRIDE_RETRACT:
            this.intakeDeployMotor.set(getRetractSpeed());
            // If the override is let go, then stop the deploy
            deployIntakeState = DeployState.STOPPED;
            break;

        case POSITION_45:
            if (Hardware.leftOperator.getRawButton(2) == false)
                {
                deployIntakeState = DeployState.DEPLOYING;
                }

            // If the deploy has reached the position, send constant voltage
            if (intakeDeployEncoder.get() < DEPLOY_45_POSITION_TICKS)
                {
                intakeDeployMotor.set(DEPLOY_HOLDING_VOLTAGE);
                }
            // If we have gone too far
            else if (intakeDeployEncoder
                    .get() < DEPLOY_45_POSITION_TICKS
                            - DEPLOY_DEADBAND)
                {
                intakeDeployMotor.stopMotor();
                }
            // We are still retracting.
            else
                {
                intakeDeployMotor.set(getRetractSpeed());
                }
            // Only set it while we are holding the button.
            deployIntakeState = DeployState.DEPLOYING;

            // if (this.timedRetract45() == true)
            // {
            // Hardware.intakeDeployArm.set(DEPLOY_HOLDING_VOLTAGE);
            // deployIntakeState = DeployState.HOLDING_45;
            // }

            break;


        case HOLDING_45:

            Hardware.intakeDeployArm.set(DEPLOY_HOLDING_VOLTAGE);
            System.out.println("HOLDING 45");
            break;


        case DEPLOY_45:
            System.out.println("DEPLOY 45");
            if (CubeManipulator.timedDeploy45() == true)
                {
                Hardware.intakeDeployArm.set(INTAKE_DEPLOY_SPEED);
                deployIntakeState = DeployState.DEPLOYED;
                }


            break;



        case FOLD_ARM_DOWN:

            // Sets up a state machine INSIDE the state machine to fold the arm
            // down to climb.
            switch (foldDownDeployState)
                {
                case RAISE_ARM:
                    // Raise the arm up and out of the way
                    if (this.getForkliftHeight() > FORKLIFT_DEPLOY_FOLDED_MIN_HEIGHT
                            || setLiftPosition(
                                    FORKLIFT_DEPLOY_FOLDED_MIN_HEIGHT) == true)
                        foldDownDeployState = FoldDownDeployState.RELEASE_TENSION;
                    break;
                // First bring the arm up halfway to relieve tension on the
                // servo arm
                case RELEASE_TENSION:
                    // If we have reached the up position?
                    if (getIntakeAngle() < INTAKE_RETRACT_TICKS)
                        {
                        // Set the servo to the "in" position get ready to bring
                        // the arm down.
                        deployFoldingServo.set(DEPLOY_SERVO_IN);
                        this.foldDownDeployState = FoldDownDeployState.FOLD_DOWN;
                        }
                    // If we have not yet reached the 45 point?
                    else
                        {
                        // Keep brining the arm up
                        this.intakeDeployMotor
                                .set(getRetractSpeed());
                        }
                    break;
                case FOLD_DOWN:
                    // We have finished going down?
                    if (getIntakeAngle() > INTAKE_FOLDED_TICKS)
                        {
                        // Reset the state for next time and change the overall
                        // deploy state.
                        this.foldDownDeployState = FoldDownDeployState.RAISE_ARM;
                        CubeManipulator.deployIntakeState = DeployState.FOLDED;
                        }
                    // We have NOT finished going down?
                    else
                        {
                        // Keep bringing the arm down
                        this.intakeDeployMotor.set(INTAKE_DEPLOY_SPEED);
                        }
                    break;
                default:
                    deployIntakeState = DeployState.STOPPED;
                }
            break;
        case FOLDED:
            intakeDeployMotor.stopMotor();
            break;

        case UNFOLD_ARM_UP:
            // If the arm has reached the 45 position
            this.deployFoldingServo.set(DEPLOY_SERVO_OUT);
            if (getIntakeAngle() < INTAKE_RETRACT_TICKS)
                {
                // set the servo out and begin the normal deploy motor code
                this.deployFoldingServo.set(DEPLOY_SERVO_OUT);
                this.deployCubeIntake(false);
                }
            else
                {
                this.intakeDeployMotor.set(getRetractSpeed());
                }
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

    if (this.isLowEject == true)
        {
        this.currentEjectSpeed = EJECT_SPEED_DROP;
        }
    else if (CubeManipulator.deployIntakeState == DeployState.POSITION_45)
        {
        this.currentEjectSpeed = EJECT_SPEED_45;
        }
    else if (this.forkliftEncoder.getDistance() > EJECT_CHANGE_HEIGHT)
        {
        this.currentEjectSpeed = EJECT_SPEED_FAST;
        }
    else
        {
        this.currentEjectSpeed = EJECT_SPEED_SLOW;
        }

    switch (intakeState)
        {
        case PULL_IN:
            // // We have a cube? stop pulling in.
            // if (hasCube() == true && intakeOverride == false)
            // this.intakeMotor.stopMotor();
            // // Don't have a cube? keep pulling in.
            // else
            this.intakeMotor.set(INTAKE_SPEED);

            // Set to stop when they stop hitting the button.
            intakeState = IntakeState.STOP;
            break;
        case PUSH_OUT:
            this.intakeMotor.set(currentEjectSpeed);
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
MOVING_TO_POSITION, STAY_AT_POSITION, MOVE_JOY, AVOID_DRAG
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
public static enum DeployState
    {
NOT_DEPLOYED, PRE_DEPLOYING, DEPLOYING, DEPLOYED, POSITION_45, FOLD_ARM_DOWN, RETRACTING, OVERRIDE_DEPLOY, OVERRIDE_RETRACT, STOPPED, FOLDED, UNFOLD_ARM_UP, HOLDING_45, DEPLOY_45
    }

/**
 * 
 * @author Ryan McGee
 *
 */
private static enum FoldDownDeployState
    {
RELEASE_TENSION, FOLD_DOWN, RAISE_ARM
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
MOVE_LIFT, DEPLOY_INTAKE, SPIT_OUT_CUBE, FINISHED, OVERRIDE
    }

// --------------------VARIABLES--------------------

// ================Override Related================

// ================FORKLIFT================
// Used in forkliftUpdate()
private ForkliftState liftState = ForkliftState.STAY_AT_POSITION;

// used to tell the forklift which direction it should be moving
private ForkliftDirectionState forkliftDirection = ForkliftDirectionState.NEUTRAL;

private boolean IROverride = false;

private boolean isClimbing = false;

private double currentForkliftDownSpeed = 0;

public double currentForkliftMaxHeight = 0; // @ANE change to private

private double forkliftTargetHeight = 0.0;

private double forkliftTargetSpeed = 0.0;

private double currentMinLiftPosition = 0;
// ========================================

// ================INTAKE==================
// private boolean deployedArm = false;

// variable that controls the deploy intake state machine
public static DeployState deployIntakeState = DeployState.NOT_DEPLOYED;

private FoldDownDeployState foldDownDeployState = FoldDownDeployState.RAISE_ARM;

private pushOutState pushState = pushOutState.INIT;

private IntakeState intakeState = IntakeState.STOP;

private double currentEjectSpeed = -1;

private double currentRetractSpeed = 0;

private boolean isPullingIn = true;

private boolean lastIntakeButtonStatus = false;

private boolean intakeOverride = false;

private boolean isRunningPushOutCubeAuto = false;

private boolean intakeArmHasBeenDroppedToClimb = false;

// ========================================

private scoreScaleState scaleState = scoreScaleState.MOVE_LIFT;

// -------------------------------------------------

// --------------------CONSTANTS--------------------

// ================FORKLIFT================

private final double FORKLIFT_MAX_HEIGHT = 69; // Changed 3/22/18 from 76

private final double FORKLIFT_DOWN_JOYSTICK_SCALAR = .55;

private final double FORKLIFT_CLIMB_SCALAR = .8;

private final double FORKLIFT_UP_JOYSTICK_SCALAR = 1.0;

private final double FORKLIFT_NO_CUBE_MIN_HEIGHT = 0;

private final double FORKLIFT_DEPLOY_FOLDED_MIN_HEIGHT = 15;

private final double FORKLIFT_DEFAULT_SPEED_UP = FORKLIFT_UP_JOYSTICK_SCALAR;

private final double FORKLIFT_DEFAULT_SPEED_DOWN = FORKLIFT_DOWN_JOYSTICK_SCALAR;

private final double FORKLIFT_STAY_UP_SPEED = 0.05; // -.15;

private final double FORKLIFT_STAY_UP_WITH_CUBE = .1;

// height above which the armIR's input will start mattering
private final double USE_ARM_IR_HEIGHT = 35.0;

public final static double SWITCH_HEIGHT = 26;

public final static double SCALE_HEIGHT = 69;// Changed 3/22/18 from 76

// =========================================

// ================INTAKE===================

private final double INTAKE_SPEED = .35;

private final double EJECT_SPEED_45 = -1;

private final double EJECT_SPEED_FAST = -.7;

private final double EJECT_SPEED_SLOW = -.45;

private final double EJECT_SPEED_DROP = -.3;

private final double EJECT_CHANGE_HEIGHT = 58;

public final double INTAKE_STOP_WITH_CUBE = .06;

// how many degrees the intake deploy motor needs to turn for the intake
// to be fully deployed
private final static double INTAKE_DEPLOY_TICKS = 240;

// the encoder value that counts as the intake being retracted
private final double INTAKE_RETRACT_TICKS = 0.0;

// Set at 180 degrees instead of 90 degrees, hence double the value of
// deployed.
private final double INTAKE_FOLDED_TICKS = 365;// 300;

// Servo is IN, deploy will be able to fold down.
public final static double DEPLOY_SERVO_IN = .4;

// Servo is OUT, deploy will be supported by servo.
public final static double DEPLOY_SERVO_OUT = .9;

// @ANE

public final double TIME_TO_DEPLOY_90 = .4;

public final double TIME_TO_RETRACT_90 = .4;

public final double TIME_TO_RETRACT_45 = .2;

public final static double TIME_TO_DEPLOY_45 = .2;

private final static double INTAKE_DEPLOY_SPEED = .3;

// speed we retract the intake mechanism at
private final double INTAKE_RETRACT_SPEED_LOW = -.5;

private final double INTAKE_RETRACT_SPEED_HIGH = -.9;

private final double DEPLOY_HOLDING_VOLTAGE = -.15;

private final double EJECT_TIME = 2.0;

// number of degrees to retract before deploying, calculated from ticks
private final double DEGREES_TO_RETRACT_TO_DEPLOY = 0.0
        * (INTAKE_DEPLOY_TICKS / 90);

private final static double DEPLOY_45_POSITION_TICKS = .5
        * INTAKE_DEPLOY_TICKS;// 160;

private final int DEPLOY_DEADBAND = 15;

public static final DoubleSolenoid.Value INTAKE_ARMS_OPEN = DoubleSolenoid.Value.kForward;

public static final DoubleSolenoid.Value INTAKE_ARMS_CLOSED = DoubleSolenoid.Value.kReverse;

// =========================================

private static final double JOYSTICK_DEADBAND = .2;

// ---------------------------------------------
}
