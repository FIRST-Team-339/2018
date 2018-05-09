package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import org.usfirst.frc.team339.Utils.transmission.TransmissionBase;
import org.usfirst.frc.team339.Utils.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.Utils.KilroyPID;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An extension of the Drive class that overrides the methods to use PID loops
 * instead of static-correcting methods.
 * 
 * @author Ryan McGee
 *
 */
public class DrivePID extends Drive
{

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

		this.encoderPID = new KilroyPID[4];
		encoderPID[0] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_REAR), leftRearEncoder);
		encoderPID[1] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_REAR), rightRearEncoder);
		encoderPID[2] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_FRONT), leftFrontEncoder);
		encoderPID[3] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_FRONT), rightFrontEncoder);

		this.gyroPID = new KilroyPID[4];
		gyroPID[0] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_REAR), gyro);
		gyroPID[1] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_REAR), gyro);
		gyroPID[2] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_FRONT), gyro);
		gyroPID[3] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_FRONT), gyro);

		this.encoders = new Encoder[4];
		encoders[0] = leftRearEncoder;
		encoders[1] = rightRearEncoder;
		encoders[2] = leftFrontEncoder;
		encoders[3] = rightFrontEncoder;
	}

	/**
	 * Create the DrivePID Object with a 2 encoder system
	 * 
	 * @param transmission
	 * @param leftEncoder
	 * @param rightEncoder
	 * @param gyro
	 */
	public DrivePID(TransmissionBase transmission, Encoder leftEncoder, Encoder rightEncoder, GyroBase gyro)
	{
		super(transmission, leftEncoder, rightEncoder, gyro);
		encoderPID = new KilroyPID[2];
		encoderPID[0] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_REAR), leftEncoder);
		encoderPID[1] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_REAR), rightEncoder);
		this.encoders = new Encoder[2];
		this.encoders[0] = leftEncoder;
		this.encoders[1] = rightEncoder;
		this.gyroPID = new KilroyPID[2];
		this.gyroPID[0] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_REAR), gyro);
		this.gyroPID[1] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_REAR), gyro);
	}

	/**
	 * Resets the all drive function's initialization.
	 */
	public void reset()
	{
		super.reset();
		this.turnDegreesInit = true;
		this.turnDegreesGyroInit = true;
		for (KilroyPID pid : encoderPID)
			pid.setEnabled(false);
		// this.turnGyroPID.disable();
	}

	/**
	 * Turns the robot X degrees on the spot, using left and right encoders.
	 * 
	 * @param degrees
	 *            How far the robot should turn, in degrees. Positive for clockwise,
	 *            negative for counterclockwise.
	 * 
	 * @param speed
	 *            How fast the robot should turn, in positive percentage.
	 * 
	 * @return whether or not the robot has finished turning.
	 */
	public boolean turnDegrees(int degrees, double speed)
	{
		// Reset the PID loop and set the PID values if first start
		if (turnDegreesInit == true)
		{
			// Reset the sensors
			super.resetEncoders();
			for (KilroyPID pid : encoderPID)
			{
				// Set PID settings
				pid.resetPID();
				pid.setPIDF(turnPIDTolerance[0], turnPIDTolerance[1], turnPIDTolerance[2], 0);
				pid.setTolerance(turnPIDTolerance[3]);
			}
			// Set the setpoint to each motor. If it's even (0 or 2), then it's
			// the
			// left side of the robot. Odds (1 or 3) are right.
			for (int i = 0; i < encoderPID.length; i++)
			{
				if (i % 2 == 0)
					encoderPID[i].setSetpoint(degreesToEncoderInches(degrees, false), false);
				else
					encoderPID[i].setSetpoint(-degreesToEncoderInches(degrees, false), false);
				// Left or right, set them to enabled.
				encoderPID[i].setSpeed(speed);
				encoderPID[i].setEnabled(true);
			}
			turnDegreesInit = false;
		}

		// If ANY motor is not on target, then keep running.
		boolean isOnTarget = true;
		for (KilroyPID pid : encoderPID)
			if (pid.isOnTarget() == false)
			{
				isOnTarget = false;
				break;
			}

		if (isOnTarget == true)
		{
			// We have reached the destination.
			for (KilroyPID pid : encoderPID)
				pid.setEnabled(false);

			turnDegreesInit = true;
			return true;
		}
		return false;
	}

	/**
	 * Turns the robot on the spot based on the gyro as a sensor, rather than the encoders.
	 * 
	 * @param degrees how far the robot should turn. Positive for clockwise, negative for counterclockwise.
	 * 
	 * @param speed the maximum speed that the robot is allowed to travel at.
	 * 
	 * @return whether or not the robot has finished turning
	 */
	public boolean turnDegreesGyro(int degrees, double speed)
	{
		if (turnDegreesGyroInit == true)
		{
			// Reset the Sensor \ PID controller
			this.getGyro().reset();
			for (int i = 0; i < this.gyroPID.length; i++)
			{
				// Reset PID accumulated values, setup P, I, D and Tolerance,
				// set setpoint, and enable PID.
				this.gyroPID[i].resetPID();
				this.gyroPID[i].setPIDF(this.turnGyroPIDTolerance[0], this.turnGyroPIDTolerance[1],
						this.turnGyroPIDTolerance[2], 0);
				this.gyroPID[i].setTolerance(this.turnGyroPIDTolerance[3]);
				if (i % 2 == 0)
					// Left side goes forward if degrees is positive, right goes
					// backwards.
					this.gyroPID[i].setSetpoint(degrees, false);
				else
					this.gyroPID[i].setSetpoint(degrees, true);
				this.gyroPID[i].setEnabled(true);
			}

			this.turnDegreesGyroInit = false;
		}

		// If any one PID loop is not on target, then keep running.
		boolean isOnTarget = false;
		for (KilroyPID pid : gyroPID)
			if (pid.isOnTarget() == false)
			{
				isOnTarget = false;
				break;
			}

		// If the robot has reached it's angle, then we are good.
		if (isOnTarget == true)
		{
			// Turn off the PID loop and return true.
			this.turnDegreesGyroInit = true;
			for (KilroyPID pid : gyroPID)
				pid.setEnabled(false);
			// We have finished turning! Yay!
			return true;
		}
		// We have not yet finished turning.
		return false;
	}

	/**
	 * Drives in a straight line based on encoder values.
	 * 
	 * @param speed How fast the robot will be running forwards / backwards.
	 * @param acceleration whether or not to accelerate at a fixed rate
	 */
	public void driveStraight(double speed, double acceleration)
	{
		if (System.currentTimeMillis() - driveStraightLastTime > INIT_TIMEOUT)
		{
			this.driveStraightPIDOutput = speed;
			this.driveStraightPID.getPIDController().reset();
			this.driveStraightPID.getPIDController().setPID(driveStraightPIDTolerance[0], driveStraightPIDTolerance[1],
					driveStraightPIDTolerance[2]);
			this.driveStraightPID.setSetpoint(0);
			this.driveStraightPID.enable();
		}

		super.accelerateTo(speed + driveStraightPIDOutput, speed - driveStraightPIDOutput, acceleration);

		driveStraightLastTime = System.currentTimeMillis();
	}

	/**
	 * Sets what the PID loops will be looking at while running
	 * 
	 * @param sourceType
	 *            Use kDisplacement for absolute positioning (i.e. turning to a
	 *            degree), Use kRate for speed adjustments (i.e. controlling RPM)
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

	// ======================PID Tuning=====================

	/**
	 * Tunes the PID loop for turning via encoders.
	 * 
	 * P - Proportional function: Value adds P percent per inch the encoder is off
	 * currently. I - Integral function: Value adds I percent per inch the encoder
	 * has been off cumulatively D - Derivative function: Value adds D percent per
	 * inch per second the error is changing in relation to the setpoint (slope of
	 * current position vs error get smaller as the controller slows down to reach
	 * it's target)
	 * 
	 * A high P value will give you a more aggressive correction, but may induce
	 * oscillation: Try to avoid that... A high I value will correct any long term
	 * "drifting to a side" problems, but again may induce oscillation. A high D
	 * value will give you a longer drawn out deceleration curve, and may fix
	 * oscillation, but will make the robot take longer reach it's setpoint.
	 * 
	 * A low tolerance will result in more accurate turns, but may induce a little
	 * "wiggle" at the end (which is not bad, but takes more time.)
	 * 
	 * The point of using a PID loop for turning is to increase the speed of the
	 * turn and reduce overshoot / undershoot.
	 */
	public void tunePID(PIDDriveFunction type)
	{
		switch (type)
		{
		case TURN:
			if (tuneTurnDegreesPIDInit == true)
			{
				SmartDashboard.putNumber("Turn P", turnPIDTolerance[0]);
				SmartDashboard.putNumber("Turn I", turnPIDTolerance[1]);
				SmartDashboard.putNumber("Turn D", turnPIDTolerance[2]);
				SmartDashboard.putNumber("Turn Tolerance", turnPIDTolerance[3]);
				tuneTurnDegreesPIDInit = false;
			}

			this.turnPIDTolerance[0] = SmartDashboard.getNumber("Turn P", turnPIDTolerance[0]);
			this.turnPIDTolerance[1] = SmartDashboard.getNumber("Turn I", turnPIDTolerance[1]);
			this.turnPIDTolerance[2] = SmartDashboard.getNumber("Turn D", turnPIDTolerance[2]);
			this.turnPIDTolerance[3] = SmartDashboard.getNumber("Turn Tolerance", turnPIDTolerance[3]);
			return;
		case DRIVESTRAIGHT:
			if (tuneDriveStraightPIDInit == true)
			{
				SmartDashboard.putNumber("Drive Straight P", driveStraightPIDTolerance[0]);
				SmartDashboard.putNumber("Drive Straight I", driveStraightPIDTolerance[1]);
				SmartDashboard.putNumber("Drive Straight D", driveStraightPIDTolerance[2]);
				SmartDashboard.putNumber("Drive Straight Tolerance", driveStraightPIDTolerance[3]);
				tuneDriveStraightPIDInit = false;
			}

			this.driveStraightPIDTolerance[0] = SmartDashboard.getNumber("Drive Straight P",
					driveStraightPIDTolerance[0]);
			this.driveStraightPIDTolerance[1] = SmartDashboard.getNumber("Drive Straight I",
					driveStraightPIDTolerance[1]);
			this.driveStraightPIDTolerance[2] = SmartDashboard.getNumber("Drive Straight D",
					driveStraightPIDTolerance[2]);
			this.driveStraightPIDTolerance[3] = SmartDashboard.getNumber("Drive Straight Tolerance",
					driveStraightPIDTolerance[3]);
			return;
		case TURN_GYRO:
			if (tuneTurnDegreesGyroPIDInit == true)
			{
				SmartDashboard.putNumber("Turn Gyro P", turnGyroPIDTolerance[0]);
				SmartDashboard.putNumber("Turn Gyro I", turnGyroPIDTolerance[1]);
				SmartDashboard.putNumber("Turn Gyro D", turnGyroPIDTolerance[2]);
				SmartDashboard.putNumber("Turn Gyro Tolerance", turnGyroPIDTolerance[3]);
				tuneTurnDegreesGyroPIDInit = false;
			}

			this.turnGyroPIDTolerance[0] = SmartDashboard.getNumber("Turn Gyro P", turnGyroPIDTolerance[0]);
			this.turnGyroPIDTolerance[1] = SmartDashboard.getNumber("Turn Gyro I", turnGyroPIDTolerance[1]);
			this.turnGyroPIDTolerance[2] = SmartDashboard.getNumber("Turn Gyro D", turnGyroPIDTolerance[2]);
			this.turnGyroPIDTolerance[3] = SmartDashboard.getNumber("Turn Gyro Tolerance", turnGyroPIDTolerance[3]);
			return;
		}

	}

	public enum PIDDriveFunction
	{
		TURN, TURN_GYRO, DRIVESTRAIGHT
	}

	private final PIDSubsystem driveStraightPID = new PIDSubsystem(0, 0, 0)
	{

		@Override
		protected double returnPIDInput()
		{
			double leftSide = 0;
			double rightSide = 0;
			for (int i = 0; i < encoders.length; i++)
				if (i % 2 == 0)
					leftSide += encoders[i].getDistance();
				else
					rightSide += encoders[i].getDistance();

			leftSide /= (encoders.length / 2);
			rightSide /= (encoders.length / 2);
			return leftSide - rightSide;
		}

		@Override
		protected void usePIDOutput(double output)
		{
			driveStraightPIDOutput = output;
		}

		@Override
		protected void initDefaultCommand()
		{

		}

	};

	private final KilroyPID[] encoderPID;
	private final KilroyPID[] gyroPID;
	private final Encoder[] encoders;

	// ======================Variables======================

	private boolean tuneTurnDegreesPIDInit = true;

	private double[] turnPIDTolerance =
			// {P, I, D, Tolerance}
			{ 0, 0, 0, 0 };

	private boolean turnDegreesInit = true;

	private boolean tuneTurnDegreesGyroPIDInit = true;
	private double[] turnGyroPIDTolerance =
	{ 0, 0, 0, 0 };
	private boolean turnDegreesGyroInit = true;

	private boolean tuneDriveStraightPIDInit = true;
	private double driveStraightPIDOutput = 0;
	private long driveStraightLastTime = 0;
	private double[] driveStraightPIDTolerance =
			// {P, I, D, Tolerance}
			{ 0, 0, 0, 0 };
}
