package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.TransmissionType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An extension of the Drive class that overrides the methods to use PID loops instead of static-correcting methods. 
 * 
 * @author Ryan McGee
 *
 */
public class DrivePID extends Drive
{

	private PIDController leftRearPID = null;
	private PIDController rightRearPID = null;
	private PIDController leftFrontPID = null;
	private PIDController rightFrontPID = null;

	/**
	 * Create the DrivePID Object with a 4 encoder system
	 * 
	 * @param transmission
	 * @param leftFrontEncoder
	 * @param rightFrontEncoder
	 * @param leftRearEncoder
	 * @param rightRearEncoder
	 * @param gyro
	 */
	public DrivePID(TransmissionBase transmission, Encoder leftFrontEncoder, Encoder rightFrontEncoder,
			Encoder leftRearEncoder, Encoder rightRearEncoder, GyroBase gyro)
	{
		// Create the Drive class this is extending, as to override the methods
		// to use PID instead of static constants.
		super(transmission, leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder, gyro);

		// Create all the PID controller objects, attached to their respective
		// encoder and motor controller.
		this.leftRearPID = new PIDController(0, 0, 0, 0, leftRearEncoder,
				super.getTransmission().getSpeedController(MotorPosition.LEFT_REAR));
		this.rightRearPID = new PIDController(0, 0, 0, 0, rightRearEncoder,
				super.getTransmission().getSpeedController(MotorPosition.RIGHT_REAR));
		this.leftFrontPID = new PIDController(0, 0, 0, 0, leftFrontEncoder,
				super.getTransmission().getSpeedController(MotorPosition.LEFT_FRONT));
		this.rightFrontPID = new PIDController(0, 0, 0, 0, rightFrontEncoder,
				super.getTransmission().getSpeedController(MotorPosition.RIGHT_FRONT));
		// TODO Auto-generated constructor stub
	}

	/**
	 * Create the DrivePID Object with a 2 encoder system
	 * @param transmission
	 * @param leftEncoder
	 * @param rightEncoder
	 * @param gyro
	 */
	public DrivePID(TransmissionBase transmission, Encoder leftEncoder, Encoder rightEncoder, GyroBase gyro)
	{
		super(transmission, leftEncoder, rightEncoder, gyro);

		// Create all the PID controller objects, attached to their respective
		// encoder and motor controller.
		this.leftRearPID = new PIDController(0, 0, 0, 0, leftEncoder,
				super.getTransmission().getSpeedController(MotorPosition.LEFT));
		this.rightRearPID = new PIDController(0, 0, 0, 0, rightEncoder,
				super.getTransmission().getSpeedController(MotorPosition.RIGHT));
	}

	/**
	 * Sets the PIDF values for each motor:
	 * 
	 * @param p the Proportional function - controls how much the robot will correct overall based on error
	 * @param i the Integral function - fine tunes the position of the robot based on accumulated error
	 * @param d the Derivative function - creates a curve that will prevent the robot from overshooting target
	 * @param f the Feed-Forward function - creates a minimum value the motor must be spinning
	 * @param motor Which PID should be changed
	 */
	private void setPIDFMotorValues(double p, double i, double d, double f, MotorPosition motor)
	{
		switch (motor)
		{
		// the whole right side of the robot
		case RIGHT:
			setPIDFMotorValues(p, i, d, f, MotorPosition.RIGHT_REAR);
			setPIDFMotorValues(p, i, d, f, MotorPosition.RIGHT_FRONT);
			break;
		// the whole left side of the robot
		case LEFT:
			setPIDFMotorValues(p, i, d, f, MotorPosition.LEFT_REAR);
			setPIDFMotorValues(p, i, d, f, MotorPosition.LEFT_FRONT);
			break;
		case RIGHT_REAR:
			if (rightRearPID != null)
				rightRearPID.setPID(p, i, d, f);
			break;
		case LEFT_REAR:
			if (leftRearPID != null)
				leftRearPID.setPID(p, i, d, f);
			break;
		case RIGHT_FRONT:
			if (rightFrontPID != null)
				rightFrontPID.setPID(p, i, d, f);
			break;
		case LEFT_FRONT:
			if (leftFrontPID != null)
				rightFrontPID.setPID(p, i, d, f);
			break;
		default:
			System.out.println("Error in setPIDMotorValues(): Motor Position does not exist.");

		}
	}

	/**
	 * Sets the PID controller in the given position.
	 * @param p the Proportional function - controls how much the robot will correct overall based on error
	 * @param i the Integral function - fine tunes the position of the robot based on accumulated error
	 * @param d the Derivative function - creates a curve that will prevent the robot from overshooting target
	 * @param motor Which PID should be changed
	 */
	private void setPIDMotorValues(double p, double i, double d, MotorPosition motor)
	{
		this.setPIDFMotorValues(p, i, d, 0, motor);
	}

	/**
	 * Resets the error on the chosen PID('s) for use in a new method, and disables them.
	 * @param pid Which PID controller should be reset
	 * 			
	 */
	private void resetPID(MotorPosition pid)
	{
		switch (pid)
		{
		// Reset the left and right sides of the robot
		case ALL:
			this.resetPID(MotorPosition.LEFT);
			this.resetPID(MotorPosition.RIGHT);
			break;
		// Reset only the left side motor PID's
		case LEFT:
			this.resetPID(MotorPosition.LEFT_FRONT);
			this.resetPID(MotorPosition.LEFT_REAR);
			break;
		// Reset only the right side motor PID's
		case RIGHT:
			this.resetPID(MotorPosition.RIGHT_FRONT);
			this.resetPID(MotorPosition.RIGHT_REAR);
			break;
		case RIGHT_REAR:
			if (rightRearPID != null)
				this.rightRearPID.reset();
			break;
		case LEFT_REAR:
			if (leftRearPID != null)
				this.leftRearPID.reset();
			break;
		case RIGHT_FRONT:
			if (rightFrontPID != null)
				this.rightFrontPID.reset();
		case LEFT_FRONT:
			if (leftFrontPID != null)
				this.leftFrontPID.reset();
			break;
		default:
			System.out.println("Error in resetPID(): Unkown state.");

		}
	}

	/**
	 * Sets what the PID loops will be looking at while running
	 * @param sourceType Use kDisplacement for absolute positioning (i.e. turning to a degree), Use kRate for speed adjustments (i.e. controlling RPM)
	 */
	private void setAllPIDSourceTypes(PIDSourceType sourceType)
	{
		// Rear Encoders or both encoders if 2 wheel drive
		if (getEncoder(MotorPosition.LEFT_REAR) != null)
			getEncoder(MotorPosition.LEFT_REAR).setPIDSourceType(sourceType);
		if (getEncoder(MotorPosition.RIGHT_REAR) != null)
			getEncoder(MotorPosition.RIGHT_REAR).setPIDSourceType(sourceType);
		// Front encoders, if they exist
		if (getEncoder(MotorPosition.LEFT_FRONT) != null)
			getEncoder(MotorPosition.LEFT_FRONT).setPIDSourceType(sourceType);
		if (getEncoder(MotorPosition.RIGHT_FRONT) != null)
			getEncoder(MotorPosition.RIGHT_FRONT).setPIDSourceType(sourceType);
	}

	/**
	 * Sets close the error should be for the PID loops to read that they are "on target"
	 * @param tolerance A number in the units appropriate for the sensor used (degrees for gyro, inches for encoders, etc.)
	 */
	private void setAllPIDTolerances(double tolerance)
	{
		if (leftFrontPID != null)
			leftFrontPID.setAbsoluteTolerance(tolerance);
		if (rightFrontPID != null)
			rightFrontPID.setAbsoluteTolerance(tolerance);
		if (leftRearPID != null)
			leftRearPID.setAbsoluteTolerance(tolerance);
		if (rightRearPID != null)
			rightRearPID.setAbsoluteTolerance(tolerance);
	}

	/**
	 * Enables all PID loops, causing them to start turning towards their setpoint give the P, I, D, and F values
	 */
	private void enableAllPIDs()
	{
		if (leftFrontPID != null)
			leftFrontPID.enable();
		if (rightFrontPID != null)
			rightFrontPID.enable();
		if (leftRearPID != null)
			leftRearPID.enable();
		if (rightRearPID != null)
			rightRearPID.enable();
	}

	/**
	 * Turns off all the PID loops, stopping all movement in the motors and ignores error in the wheel.
	 */
	private void disableAllPIDs()
	{
		if (leftFrontPID != null)
			leftFrontPID.disable();
		if (rightFrontPID != null)
			rightFrontPID.disable();
		if (leftRearPID != null)
			leftRearPID.disable();
		if (rightRearPID != null)
			rightRearPID.disable();
	}

	/**
	 * Sets the setpoint for the given PIDController
	 * @param setPoint The target position / velocity for the PIDController
	 * @param pid Which PIDController(s) should be changed
	 */
	private void setSetpoint(double setPoint, MotorPosition pid)
	{
		switch (pid)
		{
		// Reset the left and right sides of the robot
		case ALL:
			this.setSetpoint(setPoint, MotorPosition.LEFT);
			this.setSetpoint(setPoint, MotorPosition.RIGHT);
			break;
		// Reset only the left side motor PID's
		case LEFT:
			this.setSetpoint(setPoint, MotorPosition.LEFT_FRONT);
			this.setSetpoint(setPoint, MotorPosition.LEFT_REAR);
			break;
		// Reset only the right side motor PID's
		case RIGHT:
			this.setSetpoint(setPoint, MotorPosition.RIGHT_FRONT);
			this.setSetpoint(setPoint, MotorPosition.RIGHT_REAR);
			break;
		case RIGHT_REAR:
			if (rightRearPID != null)
				this.rightRearPID.setSetpoint(setPoint);
			break;
		case LEFT_REAR:
			if (leftRearPID != null)
				this.leftRearPID.setSetpoint(setPoint);
			break;
		case RIGHT_FRONT:
			if (rightFrontPID != null)
				this.rightFrontPID.setSetpoint(setPoint);
		case LEFT_FRONT:
			if (leftFrontPID != null)
				this.leftFrontPID.setSetpoint(setPoint);
			break;
		default:
			System.out.println("Error in setSetpoint(): Unkown state.");

		}
	}

	@Override
	/**
	 * Turns a certain amount of degrees based on the angle input and speed, compared to the encoder values.
	 * 
	 * @param angle how far the robot must turn: Positive for clockwise, negative for counterclockwise
	 * @param speed how fast the robot must turn, in decimal percentage (0.0 to 1.0)
	 */
	public boolean turnDegrees(int angle, double speed)
	{
		// Reset everything on the first start, and begin the PID loop.
		if (turnDegreesInit == true)
		{
			// Reset PID and sensors on the first start.
			super.resetEncoders();
			this.setAllPIDSourceTypes(PIDSourceType.kDisplacement);

			this.resetPID(MotorPosition.ALL);
			this.setPIDMotorValues(speed * turnP, speed * turnI, speed * turnD, MotorPosition.ALL);
			this.setAllPIDTolerances(super.degreesToEncoderInches(turnTolerance, false));

			if (angle < 0)
			{
				// Turning left is negative inches for the left side, positive
				// inches for the right.
				this.setSetpoint(super.degreesToEncoderInches(angle, false), MotorPosition.RIGHT);
				this.setSetpoint(-super.degreesToEncoderInches(angle, false), MotorPosition.LEFT);
			} else if (angle > 0)
			{
				// Turning right is negative inches for the right side, negative
				// inches for the left.
				this.setSetpoint(-super.degreesToEncoderInches(angle, false), MotorPosition.RIGHT);
				this.setSetpoint(super.degreesToEncoderInches(angle, false), MotorPosition.LEFT);
			} else
				// Are we even turning at all? Dude? What was the point?
				return true;

			this.enableAllPIDs();

			turnDegreesInit = false;
		}

		// If all the motors are within their tolerance, declare that we have
		// reached the degree we wanted
		if (leftRearPID.onTarget() && rightRearPID.onTarget()
				&& (super.getTransmission().getType() == TransmissionType.TRACTION
						|| (leftFrontPID.onTarget() && rightFrontPID.onTarget())))
		{
			this.disableAllPIDs();
			super.getTransmission().stop();
			turnDegreesInit = true;
			return true;
		}

		return false;
	}

	/**
	 * Sets the PID values for the turnDegrees functions.
	 * 
	 * @param p the Proportional function - controls how much the robot will correct overall based on error
	 * @param i the Integral function - fine tunes the position of the robot based on accumulated error
	 * @param d the Derivative function - creates a curve that will prevent the robot from overshooting target
	 */
	public void setTurningPIDValues(double p, double i, double d)
	{
		this.turnP = p;
		this.turnI = i;
		this.turnD = d;
	}

	/**
	 * Sets the PID tolerance for the drive motors while turning.
	 * @param tolerance A value, in degrees, where the motors should say: "I'm on target!"
	 */
	public void setTurningPIDTolerence(double tolerance)
	{
		this.turnTolerance = tolerance;
	}

	/**
	 * Drive in a straight line using encoders attached to a PID loop. Encoders MUST be reset before this can be called,
	 *  or else it will veer off into the direction it was reset in.
	 * 
	 * @param speed How fast the robot should be traveling, in percent
	 * 
	 * @param accelerate Whether or not the robot should accelerate to x speed over time.
	 */
	@Override
	public void driveStraight(double speed, boolean accelerate)
	{
		if (System.currentTimeMillis() - driveStraightTime > INIT_TIMEOUT)
		{
			// Setup the PID loop to keep the wheels turning at a constant rate
			this.setAllPIDSourceTypes(PIDSourceType.kDisplacement);
			this.resetPID(MotorPosition.ALL);

			this.setAllPIDTolerances(0.0);
			this.setPIDFMotorValues(driveStraightP, driveStraightI, driveStraightD, speed, MotorPosition.ALL);
			this.enableAllPIDs();
		}

		// Set the setpoint to the average encoder value, so if one side is more
		// than the other, the robot will compensate.
		if (accelerate == true)
			this.setSetpoint(accelerateTo(super.getEncoderDistanceAverage(WheelGroups.ALL)), MotorPosition.ALL);
		else
			this.setSetpoint(super.getEncoderDistanceAverage(WheelGroups.ALL), MotorPosition.ALL);

		driveStraightTime = System.currentTimeMillis();
	}

	/**
	 * Accelerates the speed input from 0 to the speed input over defaultAcceleration seconds.
	 * @param speed How fast the robot should be moving after acceleration is over
	 * @return the output speed that should be fed to the motors 
	 */
	private double accelerateTo(double speed)
	{
		// Avoid logical and divide by 0 errors
		if (speed == 0 || super.getDefaultAcceleration() == 0)
			return 0;

		if (System.currentTimeMillis() - accelerateToTimeout > INIT_TIMEOUT)
		{
			accelTimeElapsed = System.currentTimeMillis();
		}

		double output = speed * Math.min(((accelTimeElapsed / 1000) / super.getDefaultAcceleration()), 1.0);

		accelerateToTimeout = System.currentTimeMillis();
		return output;
	}

	// ======================PID Tuning=====================

	/**
	 * Tunes the PID loop for turning via encoders.
	 * 
	 * P - Proportional function: Value adds P percent per inch the encoder is off currently.
	 * I - Integral function: Value adds I percent per inch the encoder has been off cumulatively
	 * D - Derivative function: Value adds D percent per inch per second the error is changing in relation to the
	 * 	   setpoint (slope of current position vs error get smaller as the controller slows down to reach it's target)
	 * 
	 * A high P value will give you a more aggressive correction, but may induce oscillation: Try to avoid that...
	 * A high I value will correct any long term "drifting to a side" problems, but again may induce oscillation.
	 * A high D value will give you a longer drawn out deceleration curve, and may fix oscillation, but will make the
	 * 			robot take longer reach it's setpoint.
	 * 
	 * A low tolerance will result in more accurate turns, but may induce a little "wiggle" at the end 
	 * (which is not bad, but takes more time.)
	 * 
	 * The point of using a PID loop for turning is to increase the speed of the turn and reduce overshoot / undershoot.
	 */
	public void tuneTurnDegreesPID()
	{
		if (tuneTurnDegreesPIDInit == true)
		{
			SmartDashboard.putNumber("Turn P", turnP);
			SmartDashboard.putNumber("Turn I", turnI);
			SmartDashboard.putNumber("Turn D", turnD);
			SmartDashboard.putNumber("Turn Tolerance", turnTolerance);
			tuneTurnDegreesPIDInit = false;
		}

		this.turnP = SmartDashboard.getNumber("Turn P", turnP);
		this.turnI = SmartDashboard.getNumber("Turn I", turnI);
		this.turnD = SmartDashboard.getNumber("Turn D", turnD);
		this.turnTolerance = SmartDashboard.getNumber("Turn Tolerance", turnTolerance);
	}

	/**
	 * Tunes the PID loop for driving straight via encoders.
	 * 
	 * P - Proportional function: Value adds P percent per inch the encoder is off currently.
	 * I - Integral function: Value adds I percent per inch the encoder has been off cumulatively
	 * D - Derivative function: Value adds D percent per inch per second the error is changing in relation to the
	 * 	   setpoint (slope of current position vs error get smaller as the controller slows down to reach it's target)
	 * 
	 * The speed input into the method is 'F',  the feed forward value. This is the base speed, and the PID determines
	 * what is added / subtracted to each side to keep the robot correcting.
	 * 
	 * A high P value will give you a more aggressive correction, but may induce oscillation: Try to avoid that...
	 * A high I value will correct any long term "drifting to a side" problems, but again may induce oscillation.
	 * A high D value will give you a longer drawn out deceleration curve, and may fix oscillation, but will make the
	 * 			robot take longer reach it's setpoint.
	 * 
	 * The point of using a PID loop for turning is to increase the speed of the turn and reduce overshoot / undershoot.
	 */
	public void tuneDriveStraightPID()
	{
		if (tuneDriveStraightPIDInit == true)
		{
			SmartDashboard.putNumber("Drive P", driveStraightP);
			SmartDashboard.putNumber("Drive I", driveStraightI);
			SmartDashboard.putNumber("Drive D", driveStraightD);
			tuneDriveStraightPIDInit = false;
		}
		this.driveStraightP = SmartDashboard.getNumber("Drive P", driveStraightP);
		this.driveStraightI = SmartDashboard.getNumber("Drive I", driveStraightI);
		this.driveStraightD = SmartDashboard.getNumber("Drive D", driveStraightD);
	}

	// ======================Variables======================

	private double turnP = 0, turnI = 0, turnD = 0, turnTolerance = 0;

	private double driveStraightP = 0, driveStraightI = 0, driveStraightD = 0;

	private boolean turnDegreesInit = true;

	private boolean tuneTurnDegreesPIDInit = true;

	private boolean tuneDriveStraightPIDInit = true;

	private long driveStraightTime = 0;

	private long accelerateToTimeout = 0;

	private long accelTimeElapsed = 0;
}
