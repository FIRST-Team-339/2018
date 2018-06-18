package org.usfirst.frc.team339.Utils.drive;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyEncoder;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.Utils.KilroyPID;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

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
	public DrivePID(TransmissionBase transmission, KilroyEncoder leftFrontEncoder, KilroyEncoder rightFrontEncoder,
			KilroyEncoder leftRearEncoder, KilroyEncoder rightRearEncoder, GyroBase gyro)
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

		this.encoders = new KilroyEncoder[4];
		encoders[0] = leftRearEncoder;
		encoders[1] = rightRearEncoder;
		encoders[2] = leftFrontEncoder;
		encoders[3] = rightFrontEncoder;

		driveStraightInchesPID.setName("Drive Straight Inches");
		driveStraightPID_enc.setName("Encoder Drive Straight");
		driveStraightPID_gyro.setName("Gyro Drive Straight");

	}

	/**
	 * Create the DrivePID Object with a 2 encoder system
	 * 
	 * @param transmission
	 * @param leftEncoder
	 * @param rightEncoder
	 * @param gyro
	 */
	public DrivePID(TransmissionBase transmission, KilroyEncoder leftEncoder, KilroyEncoder rightEncoder, GyroBase gyro)
	{
		super(transmission, leftEncoder, rightEncoder, gyro);
		encoderPID = new KilroyPID[2];
		encoderPID[0] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_REAR), leftEncoder);
		encoderPID[1] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_REAR), rightEncoder);
		this.encoders = new KilroyEncoder[2];
		this.encoders[0] = leftEncoder;
		this.encoders[1] = rightEncoder;
		this.gyroPID = new KilroyPID[2];
		this.gyroPID[0] = new KilroyPID(transmission.getSpeedController(MotorPosition.LEFT_REAR), gyro);
		this.gyroPID[1] = new KilroyPID(transmission.getSpeedController(MotorPosition.RIGHT_REAR), gyro);
	}

	/**
	 * Resets the all drive function's initialization.
	* 
	 * @return 
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
	 * Sets the p, i, and d of the drive function selected for autonomous driving.
	 * 
	 * @param driveFunction
	 *            Which PID loop will be tuned
	 * @param p
	 *            the Proportional value
	 * @param i
	 *            the Integral value
	 * @param d
	 *            the Derivative value
	 * @param tolerance 
	*            How far when we are considered "on target", in default sensor
	*            units.
	 */
	public void setPIDTolerance(PIDDriveFunction driveFunction, double p, double i, double d, double tolerance)
	{
		switch (driveFunction)
		{
		case DRIVESTRAIGHT_ENC:
			this.driveStraightPIDTolerance[0] = p;
			this.driveStraightPIDTolerance[1] = i;
			this.driveStraightPIDTolerance[2] = d;
			this.driveStraightPIDTolerance[3] = tolerance;
			return;
		case DRIVESTRAIGHT_GYRO:
			this.driveStraightGyroPIDTolerance[0] = p;
			this.driveStraightGyroPIDTolerance[1] = i;
			this.driveStraightGyroPIDTolerance[2] = d;
			this.driveStraightGyroPIDTolerance[3] = tolerance;
			return;
		case TURN_ENC:
			this.turnPIDTolerance[0] = p;
			this.turnPIDTolerance[1] = i;
			this.turnPIDTolerance[2] = d;
			this.turnPIDTolerance[3] = tolerance;
			return;
		case TURN_GYRO:
			this.turnGyroPIDTolerance[0] = p;
			this.turnGyroPIDTolerance[1] = i;
			this.turnGyroPIDTolerance[2] = d;
			this.turnGyroPIDTolerance[3] = tolerance;
			return;
		case DRIVESTRAIGHTINCHES:
			this.driveStraightInchesPIDTolerance[0] = p;
			this.driveStraightInchesPIDTolerance[1] = i;
			this.driveStraightInchesPIDTolerance[2] = d;
			this.driveStraightInchesPIDTolerance[3] = tolerance;
		default:
			return;
		}
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
	 * Turns the robot on the spot based on the gyro as a sensor, rather than the
	 * encoders.
	 * 
	 * @param degrees
	 *            how far the robot should turn. Positive for clockwise, negative
	 *            for counterclockwise.
	 * 
	 * @param speed
	 *            the maximum speed that the robot is allowed to travel at.
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
	 * @param speed
	 *            How fast the robot will be running forwards / backwards.
	 * @param acceleration
	 *            whether or not to accelerate at a fixed rate
	 * @param isUsingGyro
	 *            If true, then the PID loop will use a gyro as the sensor for
	 *            correction. If false, it will use encoders.
	 */
	public void driveStraight(double speed, double acceleration, boolean isUsingGyro)
	{
		if (System.currentTimeMillis() - driveStraightLastTime > INIT_TIMEOUT)
		{
			if (isUsingGyro == true)
			{
				this.driveStraightPID_gyro.getPIDController().reset();
				this.driveStraightPID_gyro.getPIDController().setPID(driveStraightGyroPIDTolerance[0],
						driveStraightGyroPIDTolerance[1], driveStraightGyroPIDTolerance[2]);
				this.driveStraightPID_gyro.setSetpoint(0);
				this.driveStraightPID_gyro.enable();
			} else
			{
				this.driveStraightPID_enc.getPIDController().reset();
				this.driveStraightPID_enc.getPIDController().setPID(driveStraightPIDTolerance[0],
						driveStraightPIDTolerance[1], driveStraightPIDTolerance[2]);
				this.driveStraightPID_enc.setSetpoint(0);
				this.driveStraightPID_enc.enable();
			}
		}
		if (isUsingGyro == true)
			super.accelerateTo(speed + driveStraightPIDOutput_gyro, speed - driveStraightPIDOutput_gyro, acceleration);
		else
			super.accelerateTo(speed + driveStraightPIDOutput_enc, speed - driveStraightPIDOutput_enc, acceleration);

		driveStraightLastTime = System.currentTimeMillis();
	}

	/**
	* Drives a set number of inches forwards in a straight line, based on encoder
	* distance, correcting for
	 * misalignment based on Gyro and encoder PID tunings.
	 * 
	* If tuned properly, this will give the robot a nice acceleration and
	* deceleration curve,
	 *  while correcting it's position.
	 * 
	 * @param speed
	 * 			The robot's maximum speed after acceleration, in percentage
	 * @param distance
	 * 			How far the robot must travel, in inches
	 * @param acceleration
	 * 			How fast to accelerate from 0, in seconds
	 * @param isUsingGyro 
	*            Whether or not the driveStraight part of this is using the gyro
	*            for correction.
	 * 			If false, the encoders are used.
	 * @return
	 * 		Whether or not the robot has reached it's destination.
	 */
	public boolean driveStraightInches(double speed, double distance, double acceleration, boolean isUsingGyro)
	{
		// On initialization, reset encoders and the PID controller, set
		// PIDTolerance values and setpoint,
		/// set maximum speed, and begin moving.
		if (driveStraightInchesInit == true)
		{
			resetEncoders();
			driveStraightInchesPID.getPIDController().setPID(driveStraightInchesPIDTolerance[0],
					driveStraightInchesPIDTolerance[1], driveStraightInchesPIDTolerance[2]);
			driveStraightInchesPID.setAbsoluteTolerance(driveStraightInchesPIDTolerance[3]);
			driveStraightInchesPID.getPIDController().reset();
			driveStraightInchesPID.setSetpoint(distance);
			driveStraightInchesPID.setOutputRange(-speed, speed);
			driveStraightInchesPID.enable();
			driveStraightInchesInit = false;
		}

		// If we have reached the setpoint, then disable the PID loop and stop
		// moving.
		if (driveStraightInchesPID.onTarget() == true)
		{
			this.stop();
			driveStraightInchesPID.disable();
			driveStraightInchesInit = true;
			return true;
		}

		// If we have not reached the setpoint, then keep driving.
		driveStraight(this.driveStraightInchesSpeed, acceleration, isUsingGyro);

		return false;
	}

	// ======================PID Tuning=====================

	/*
	 * Tunes the PID loop for turning via encoders.
	 * 
	 * P - Proportional function: Value adds P percent per inch the encoder is
	 * off currently.
	 * 
	 * I - Integral function: Value adds I percent per inch the encoder has been
	 * off cumulatively
	 * 
	 * D - Derivative function: Value adds D percent per inch per second the
	 * error is changing in relation to the setpoint (slope of current position
	 * vs error get smaller as the controller slows down to reach it's target)
	 * 
	 * A high P value will give you a more aggressive correction, but may induce
	 * oscillation: Try to avoid that... A high I value will correct any long
	 * term "drifting to a side" problems, but again may induce oscillation. A
	 * high D value will give you a longer drawn out deceleration curve, and may
	 * fix oscillation, but will make the robot take longer reach it's setpoint.
	 * 
	 * In order to tune, start by changing P. Think of P like this: for every 1
	 * sensor unit, x percent will be added. After P, start tuning D. End with
	 * I, and only use it where you see fit. I is great for longer, more precise
	 * movements, and must be small.
	 * 
	 * A low tolerance will result in more accurate turns, but may induce a
	 * little "wiggle" at the end (which is not bad, but takes more time.)
	 * 
	 * The point of using a PID loop for turning is to increase the speed of the
	 * turn and reduce overshoot / undershoot.
	 * 
	 * ...Good luck...
	 *
	 */

	/**
	* Tunes the PID controllers for the drive fuctions of the robot
	* 
	* @param type
	*            Which driving function is being tuned
	* @return whether the PID is enabled or not.
	*/
	public boolean tunePID(PIDDriveFunction type)
	{
		switch (type)
		{
		case TURN_ENC:
			// Set the PID and tolerance values
			turnPIDTolerance = new double[]
			{ encoderPIDTuner.p, encoderPIDTuner.i, encoderPIDTuner.d, encoderPIDTuner.tolerance };
			// If enabled, then turn the x degrees. If not, stop.
			if (encoderPIDTuner.enabled)
			{
				if (this.turnDegrees((int) encoderPIDTuner.setpoint, encoderPIDTuner.speed))
				{
					stop();
					encoderPIDTuner.enabled = false;
				}

			} else
			{
				reset();
			}
			return encoderPIDTuner.enabled;
		case TURN_GYRO:
			// Set the PID and tolerance values
			turnGyroPIDTolerance = new double[]
			{ gyroPIDTuner.p, gyroPIDTuner.i, gyroPIDTuner.d, gyroPIDTuner.tolerance };
			// If enabled, then turn the x degrees. If not, stop.
			if (gyroPIDTuner.enabled)
			{
				if (this.turnDegreesGyro((int) gyroPIDTuner.setpoint, gyroPIDTuner.speed))
				{
					stop();
					gyroPIDTuner.enabled = false;
				}

			} else
			{
				reset();
			}
			return gyroPIDTuner.enabled;
		default:
			return false;
		}
	}

	/**
	 * @return Whether or not the PID loops are being tuned.
	 */
	public boolean isTuningPID()
	{
		return isTuningPID;
	}

	/**
		 * Describes the different drive-by-pid functions
		 * 
		 * @author Ryan McGee
		 */
	public enum PIDDriveFunction
	{
		/** Turning via encoders */
		TURN_ENC,
		/** Turning via gyro sensor */
		TURN_GYRO,
		/** Driving straight via encoders */
		DRIVESTRAIGHT_ENC,
		/** Driving straight via gyro sensor */
		DRIVESTRAIGHT_GYRO,
		/**Driving straight X number of inches*/
		DRIVESTRAIGHTINCHES
	}

	/**
	 * The PID controller behind the driveStraight function when using encoders
	 */
	private final PIDSubsystem driveStraightPID_enc = new PIDSubsystem(0, 0, 0)
	{

		@Override
		protected double returnPIDInput()
		{
			return getEncoderDistanceAverage(MotorPosition.LEFT) - getEncoderDistanceAverage(MotorPosition.RIGHT);
		}

		@Override
		protected void usePIDOutput(double output)
		{
			driveStraightPIDOutput_enc = output;
		}

		@Override
		protected void initDefaultCommand()
		{
		}

	};

	/**
	 * The PID controller behind the driveStraight function when using the gyroscopic sensor
	 */
	private final PIDSubsystem driveStraightPID_gyro = new PIDSubsystem(0, 0, 0)
	{
		@Override
		protected double returnPIDInput()
		{
			return getGyro().getAngle();
		}

		@Override
		protected void usePIDOutput(double output)
		{
			driveStraightPIDOutput_gyro = output;
		}

		@Override
		protected void initDefaultCommand()
		{
		}
	};

	/**
	 * The PID loop behind the driveStraightInches function, for distance
	 */
	private final PIDSubsystem driveStraightInchesPID = new PIDSubsystem(0, 0, 0)
	{

		@Override
		protected double returnPIDInput()
		{
			return (getEncoderDistanceAverage(MotorPosition.LEFT) + getEncoderDistanceAverage(MotorPosition.RIGHT))
					/ 2.0;
		}

		@Override
		protected void usePIDOutput(double output)
		{
			driveStraightInchesSpeed = output;
		}

		@Override
		protected void initDefaultCommand()
		{
		}

	};

	private final KilroyPID[] encoderPID;

	private final KilroyPID[] gyroPID;

	private final KilroyEncoder[] encoders;

	// ======================Variables======================

	private PIDTuner encoderPIDTuner = new PIDTuner("EncoderPID");

	private PIDTuner gyroPIDTuner = new PIDTuner("GyroPID");

	private boolean isTuningPID = false;

	private double[] turnPIDTolerance =
			// {P, I, D, Tolerance}
			{ 0, 0, 0, 0 };

	private boolean turnDegreesInit = true;

	private double[] turnGyroPIDTolerance =
	{ 0, 0, 0, 0 };

	private boolean turnDegreesGyroInit = true;

	private double driveStraightPIDOutput_enc = 0;

	private long driveStraightLastTime = 0;

	private double[] driveStraightPIDTolerance =
			// {P, I, D, Tolerance}
			{ 0, 0, 0, 0 };

	private double[] driveStraightGyroPIDTolerance =
	{ 0, 0, 0, 0 };

	private double driveStraightPIDOutput_gyro = 0;

	private boolean driveStraightInchesInit = true;

	private double driveStraightInchesSpeed = 0;

	private double[] driveStraightInchesPIDTolerance =
	{ 0, 0, 0, 0 };

	/**
	 * A class designed to use the Shuffleboard built in tuner for PID loops. This is useful if you have multiple PID
	 * controllers that all need the same tuning at once, so you don't have to change 24 values for a 4 wheel robot.
	 * When values are changed in the shuffleboard, the stored values p,i,d,setpoint,tolerance,speed, and enabled will
	 * reflect it.
	 * 
	 * You are able to get a specific widget from this by finding the "LiveWindow" tab on the left, and finding the 
	 * name of the PID controller (for example, "EncoderPID") and dragging it onto the screen, if there's not
	 * already a saved view for it
	 * 
	 * @author Ryan McGee
	 * @written 6/2018
	 *
	 */
	class PIDTuner
	{
		// Store the P (proportional), I (integral), D (derivative), setpoint,
		// tolerance, speed and enabled in variables inside
		// the object
		double p, i, d, setpoint, tolerance, speed;
		boolean enabled;

		/**
		 * Creates the PIDTuner class, sets the name and creates / sends the PID widget to shuffleboard.
		 * 
		 * @param name what the PID tuner will show up as in the list under "LiveWindow".
		 */
		public PIDTuner(String name)
		{
			sendable.setName(name);
		}

		// Creating the sendable creates the widget and sends the values to the
		// shuffleboard.
		SendableBase sendable = new SendableBase()
		{
			@Override
			public void initSendable(SendableBuilder builder)
			{
				// Telling shuffleboard we want this specific widget
				builder.setSmartDashboardType("PIDController");

				// the "() -> variable" is a way of sending the variable to
				// shuffleboard
				// the "(arg0) -> variable = arg0" is a way of getting the
				// variable from the shuffleboard
				builder.addDoubleProperty("P", () -> p, (arg0) -> p = arg0);
				builder.addDoubleProperty("I", () -> i, (arg0) -> i = arg0);
				builder.addDoubleProperty("D", () -> d, (arg0) -> d = arg0);
				builder.addDoubleProperty("Setpoint", () -> setpoint, (arg0) -> setpoint = arg0);
				builder.addDoubleProperty("Speed", () -> speed, (arg0) -> speed = arg0);
				builder.addDoubleProperty("Tolerance", () -> tolerance, (arg0) -> tolerance = arg0);
				builder.addBooleanProperty("Enabled", () -> enabled, (arg0) -> enabled = arg0);
			}
		};

	}

}
