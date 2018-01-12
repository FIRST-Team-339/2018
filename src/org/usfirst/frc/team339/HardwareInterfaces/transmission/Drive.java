package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyGyro;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.TransmissionType;
import org.usfirst.frc.team339.vision.VisionProcessor;

import edu.wpi.first.wpilibj.Encoder;

/**
 * The class that controls autonomous driving functions or
 * driver-assisting functions based on sensors.
 * 
 * @author Ryan McGee
 * @written 7/26/2017
 */
public class Drive
{
	// The transmission objects. Only one is used based on the transmission
	// object that is input.

	// The reason this is not one "TransmissionBase" object is that the drive
	// functions of each type require a different number of joysticks/input
	// values. Thus, inheritance is hard.
	private TankTransmission tankTransmission = null;

	private TractionTransmission tractionTransmission = null;

	private MecanumTransmission mecanumTransmission = null;

	private Encoder leftFrontEncoder = null, rightFrontEncoder = null, leftRearEncoder = null, rightRearEncoder = null;

	private UltraSonic ultrasonic = null;

	private KilroyGyro gyro = null;

	private VisionProcessor visionProcessor = null;

	private final TransmissionType transmissionType;

	/**
	 * Creates the Drive object. If a sensor listed is not used (except for
	 * encoders), set it to null.
	 * 
	 * 
	 * @param transmission
	 *            The robot's transmission object
	 * @param leftFrontEncoder
	 *            The left-front corner encoder
	 * @param rightFrontEncoder
	 *            The right-front corner encoder
	 * @param leftRearEncoder
	 *            The left-rear corner encoder
	 * @param rightRearEncoder
	 *            The right-rear corner encoder
	 * @param ultrasonic
	 *            The sensor that finds distance using sound
	 * @param gyro
	 *            A sensor that uses a spinning disk to measure rotation.
	 * @param visionProcessor
	 *            The camera's vision processing code, as a sensor.
	 */
	public Drive(TransmissionBase transmission, Encoder leftFrontEncoder, Encoder rightFrontEncoder,
			Encoder leftRearEncoder, Encoder rightRearEncoder, UltraSonic ultrasonic, KilroyGyro gyro,
			VisionProcessor visionProcessor)
	{
		this.transmissionType = transmission.getType();
		this.leftFrontEncoder = leftFrontEncoder;
		this.rightFrontEncoder = rightFrontEncoder;
		this.leftRearEncoder = leftRearEncoder;
		this.rightRearEncoder = rightRearEncoder;

		this.ultrasonic = ultrasonic;
		this.gyro = gyro;
		this.visionProcessor = visionProcessor;

		init(transmission);
	}

	/**
	 * Creates the Drive object. If a sensor listed is not used (except for
	 * encoders), set it to null.
	 * Setup for Traction drive (only 2 motors/encoders)
	 * 
	 * @param transmission
	 *            The robot's transmission object
	 * @param leftEncoder
	 *            The left-side encoder
	 * @param rightEncoder
	 *            The right-side encoder
	 * @param ultrasonic
	 *            The sensor that finds distance using sound
	 * @param gyro
	 *            A sensor that uses a spinning disk to measure rotation.
	 */
	public Drive(TransmissionBase transmission, Encoder leftEncoder, Encoder rightEncoder, UltraSonic ultrasonic,
			KilroyGyro gyro)
	{
		this.transmissionType = transmission.getType();
		this.leftRearEncoder = leftEncoder;
		this.rightRearEncoder = rightEncoder;

		this.ultrasonic = ultrasonic;
		this.gyro = gyro;

		init(transmission);
	}

	private void init(TransmissionBase transmission)
	{

		// Only sets the transmission if it is the same type. Other transmission
		// objects get set to null.
		switch (transmissionType)
		{
		case MECANUM:
			this.mecanumTransmission = (MecanumTransmission) transmission;
			break;
		case TANK:
			this.tankTransmission = (TankTransmission) transmission;
			break;
		case TRACTION:
			this.tractionTransmission = (TractionTransmission) transmission;
			break;
		default:
			System.out.println(
					"There was an error setting up the DRIVE class. Please check the declaration for a valid transmission object.");
			break;

		}
	}

	/**
	 * Checks if the value input is in between -1 and 1 to keep it in range for
	 * motor inputs.
	 * 
	 * @param val
	 *            The input value
	 * @return The correctly ranged value
	 */
	private double inRange(double val)
	{
		if (val > 1)
			return 1;
		else if (val < -1)
			return -1;

		return val;
	}

	/**
	 * Gets the transmission object stored. ONLY use it for transmission.stop()
	 * and transmission.driveRaw()
	 * 
	 * @return The current transmission object used in the Drive class
	 */
	public TransmissionBase getTransmission()
	{
		switch (transmissionType)
		{
		case MECANUM:
			return mecanumTransmission;
		case TANK:
			return tankTransmission;
		case TRACTION:
			return tractionTransmission;
		default:
			return null;
		}
	}

	// ================ENCODER METHODS================
	/**
	 * Different groups of wheels for use in encoder data collection.
	 * 
	 * @author Ryan McGee
	 */
	public enum WheelGroups
	{
		/**
		 * All wheels combined
		 */
		ALL,
		/**
		 * Both front and back on the left side
		 */
		LEFT_SIDE,
		/**
		 * Both front and back on the right side
		 */
		RIGHT_SIDE,
		/**
		 * Both left and right rear wheels
		 */
		REAR
	}

	/**
	 * Sets how far the robot has driven per pulse the encoder reads.
	 * This value should be much lower than one, as there are usually
	 * hundreds of pulses per rotation.
	 * 
	 * To calculate, reset the encoders and
	 * push the robot forwards, say, five feet. Then count the number of pulses
	 * and do: (5x12)/pulses to get this in inches.
	 * 
	 * @param value
	 *            The encoder distance per pulse.
	 * @param encoder
	 *            Which encoder will be changed
	 */
	public void setEncoderDistancePerPulse(double value, TransmissionBase.MotorPosition encoder)
	{
		switch (encoder)
		{
		case ALL:
			if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
			{
				leftFrontEncoder.setDistancePerPulse(value);
				rightFrontEncoder.setDistancePerPulse(value);
				leftRearEncoder.setDistancePerPulse(value);
				rightRearEncoder.setDistancePerPulse(value);
			} else if (transmissionType == TransmissionType.TRACTION)
			{
				leftRearEncoder.setDistancePerPulse(value);
				rightRearEncoder.setDistancePerPulse(value);
			}
			break;
		case LEFT_FRONT:
			leftFrontEncoder.setDistancePerPulse(value);
			break;
		case RIGHT_FRONT:
			rightFrontEncoder.setDistancePerPulse(value);
			break;
		case LEFT_REAR:
			leftRearEncoder.setDistancePerPulse(value);
			break;
		case RIGHT_REAR:
			rightRearEncoder.setDistancePerPulse(value);
			break;
		default:
			break;
		}
	}

	/**
	 * Sets the encoder's stored pulses back to zero.
	 */
	public void resetEncoders()
	{
		if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
		{
			leftFrontEncoder.reset();
			rightFrontEncoder.reset();
			leftRearEncoder.reset();
			rightRearEncoder.reset();
		} else if (transmissionType == TransmissionType.TRACTION)
		{
			leftRearEncoder.reset();
			rightRearEncoder.reset();
		}
	}

	/**
	 * Gets the averages of certain wheel groups. All values are the absolute value
	 * to stop
	 * negative numbers from affecting the average.
	 * 
	 * @param encoderGroup
	 * @return
	 */
	public double getEncoderDistanceAverage(WheelGroups encoderGroup)
	{
		switch (encoderGroup)
		{
		case ALL:
			return (Math.abs(leftFrontEncoder.getDistance()) + Math.abs(rightFrontEncoder.getDistance())
					+ Math.abs(leftRearEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 4.0;
		case LEFT_SIDE:
			return (Math.abs(leftFrontEncoder.getDistance()) + Math.abs(leftRearEncoder.getDistance())) / 2.0;
		case RIGHT_SIDE:
			return (Math.abs(rightFrontEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 2.0;
		case REAR:
			return (Math.abs(leftRearEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 2.0;
		default:
			return 0.0;
		}
	}

	/**
	 * Tests whether any encoder reads larger than the input length. Useful for
	 * knowing
	 * when to stop the robot.
	 * 
	 * @param length
	 *            The desired length
	 * @return True when any encoder is past length
	 */
	private boolean isAnyEncoderLargerThan(double length)
	{
		if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
			return (Math.abs(leftFrontEncoder.getDistance()) > length
					|| Math.abs(rightFrontEncoder.getDistance()) > length
					|| Math.abs(leftRearEncoder.getDistance()) > length
					|| Math.abs(rightRearEncoder.getDistance()) > length);
		return (Math.abs(leftRearEncoder.getDistance()) > length || Math.abs(rightRearEncoder.getDistance()) > length);

	}

	// ================ DRIVE METHODS ================

	/**
	 * Resets the Drive class's functions, in case they were cut short.
	 */
	public void reset()
	{
		this.brakeInit = true;
		this.driveInchesInit = true;
		this.driveStraightInchesInit = true;
		this.turnDegreesInit = true;
	}

	/**
	 * Stops the robot suddenly, to prevent drifting during autonomous functions,
	 * and increase the precision.
	 * 
	 * @return
	 *         Whether or not the robot has stopped moving.
	 */
	public boolean brake()
	{
		if (brakeInit)
		{
			// Makes sure the robot will brake straight-ish
			// by resetting EVERYTHING
			if (this.transmissionType == TransmissionType.TRACTION)
			{
				// If two wheel transmission, only use 2 values
				this.prevBrakeVals = new double[]
				{ 0, 0 };
				this.brakeMotorPower = new double[]
				{ 0, 0 };
			} else
			{
				// Use 4 values for 4 wheel transmissions
				this.prevBrakeVals = new double[]
				{ 0, 0, 0, 0 };
				this.brakeMotorPower = new double[]
				{ 0, 0, 0, 0 };
			}

			this.resetEncoders();
			this.previousBrakeTime = System.currentTimeMillis();

			brakeInit = false;
		}
		// Only test the encoders every (COLLECTION_TIME) milliseconds.
		if (System.currentTimeMillis() - this.previousBrakeTime > COLLECTION_TIME)
		{

			// Scale the power based on how much the bot has moved, and a
			// scaling factor.
			this.brakeMotorPower[0] = inRange(
					this.brakeScalar * (this.leftRearEncoder.getDistance() - this.prevBrakeVals[0]));
			this.brakeMotorPower[1] = inRange(
					this.brakeScalar * (this.rightRearEncoder.getDistance() - this.prevBrakeVals[1]));

			if (this.transmissionType != TransmissionType.TRACTION)
			{ // Only use the next 2 values if it is a 4 wheel transmission
				this.brakeMotorPower[2] = inRange(
						this.brakeScalar * (this.leftFrontEncoder.getDistance() - this.prevBrakeVals[2]));
				this.brakeMotorPower[3] = inRange(
						this.brakeScalar * (this.rightFrontEncoder.getDistance() - this.prevBrakeVals[3]));
			}

			// Make sure that if a wheel is inside the deadband, shut off power
			// to it.
			for (int i = 0; i < brakeMotorPower.length; i++)
				if (Math.abs(this.brakeMotorPower[i]) < BRAKE_DEADBAND)
					brakeMotorPower[i] = 0;

			// Store the values of the encoders for next time.
			this.prevBrakeVals[0] = this.leftRearEncoder.getDistance();
			this.prevBrakeVals[1] = this.rightRearEncoder.getDistance();

			if (this.transmissionType != TransmissionType.TRACTION)
			{// Only use the next 2 values if it is a 4 wheel transmission
				this.prevBrakeVals[2] = this.leftFrontEncoder.getDistance();
				this.prevBrakeVals[3] = this.rightFrontEncoder.getDistance();
			}

			// Only end the method when all wheels are within the deadband.
			// Use boolean short-circuiting to make sure the [2] and [3] values
			// are only checked if it is a 4 wheel drive.
			if ((prevBrakeVals[0] == 0 && prevBrakeVals[1] == 0) && (transmissionType == TransmissionType.TRACTION
					|| (prevBrakeVals[2] == 0 && prevBrakeVals[3] == 0)))
			{
				this.getTransmission().stop();
				brakeInit = true;
				return true;
			}

			// Reset the "Timer"
			this.previousBrakeTime = System.currentTimeMillis();
		}

		this.getTransmission().getSpeedController(MotorPosition.LEFT_REAR).set(-brakeMotorPower[0]);
		this.getTransmission().getSpeedController(MotorPosition.RIGHT_REAR).set(-brakeMotorPower[1]);
		if (this.transmissionType != TransmissionType.TRACTION)
		{// If four wheel drive
			this.getTransmission().getSpeedController(MotorPosition.LEFT_FRONT).set(-brakeMotorPower[2]);
			this.getTransmission().getSpeedController(MotorPosition.RIGHT_FRONT).set(-brakeMotorPower[3]);
		}

		return false;
	}

	private boolean brakeInit = true;

	private long previousBrakeTime = 1;

	private double[] brakeMotorPower =
	{ 0, 0, 0, 0 };

	private double[] prevBrakeVals =
	{ 0, 0, 0, 0 };

	private double brakeScalar = 1;

	/**
	 * Sets the constant that is multiplied by delta-encoder values when braking.
	 * 
	 * @param brakeScalar
	 *            The value to be set. Must be higher than 0.
	 */
	public void setBrakeScalingFactor(double brakeScalar)
	{
		this.brakeScalar = brakeScalar;
	}

	/**
	 * Drives the robot a certain distance without encoder correction.
	 * Not using correction increases reliability but decreases precision.
	 * If one encoder fails, it will instead look for other encoders for input.
	 * 
	 * @param distance
	 *            how far the robot should travel. Should always remain positive!
	 * @param speed
	 *            how fast the robot should go while traveling. Negative for
	 *            backwards.
	 * @return whether or not the robot has reached "distance".
	 */
	public boolean driveInches(int distance, double speed)
	{
		// Reset encoders on initialization.
		if (this.driveInchesInit == true)
		{
			this.resetEncoders();
			this.driveInchesInit = false;
		}

		// Test if ANY encoder is past the distance.
		if (this.isAnyEncoderLargerThan(distance) == true)
		{
			this.driveInchesInit = true;
			return true;
		}

		this.getTransmission().driveRaw(speed, speed);
		return false;
	}

	private boolean driveInchesInit = true;

	/**
	 * Drives the robot a certain distance based on the encoder values.
	 * If the robot should go backwards, set speed to be negative instead of
	 * distance.
	 * 
	 * @param distance
	 *            How far the robot should go (should be greater than 0)
	 * @param speed
	 *            How fast the robot should travel
	 * @return Whether or not the robot has finished traveling that given distance.
	 */
	public boolean driveStraightInches(int distance, double speed)
	{
		// Runs once when the method runs the first time, and does not run again
		// until after the method returns true.
		if (driveStraightInchesInit == true)
		{
			this.resetEncoders();
			driveStraightInchesInit = false;
		}

		// Check encoders to see if the distance has been driven
		if (this.transmissionType == TransmissionType.MECANUM || this.transmissionType == TransmissionType.TANK)
		{
			// Check all encoders if it is a four wheel drive system.
			if (this.getEncoderDistanceAverage(WheelGroups.ALL) > distance)
			{
				this.getTransmission().stop();
				driveStraightInchesInit = true;
				return true;
			}
		} else
		{
			// Only check the rear encoders if it is a two wheel drive system.
			if (this.getEncoderDistanceAverage(WheelGroups.REAR) > distance)
			{
				this.getTransmission().stop();
				driveStraightInchesInit = true;
				return true;
			}
		}

		// Drive straight if we have not reached the distance
		this.driveStraight(speed);

		return false;
	}

	private boolean driveStraightInchesInit = true;

	/**
	 * Strafe to a target using a mecanum transmission, and a gyro for stabilization.
	 * This will NOT be accurate because of mecanum's slippery properties.
	 * @param inches
	 * 			How far we should travel
	 * @param speed
	 * 			How fast we should travel, in decimal percentage (0.0 to 1.0)
	 * @param directionDegrees
	 * 			In which direction we should travel, 0 being forwards, -90 for left and 90 for right.
	 * @return
	 * 			Whether or not we have finished strafing.
	 */
	public boolean strafeStraightInches(int inches, double speed, int directionDegrees)
	{
		// Wrong transmission type!
		if (this.transmissionType != TransmissionType.MECANUM)
			return true;

		// Reset the gyro and encoders on first start only
		if (strafeStraightInchesInit)
		{
			this.resetEncoders();
			this.gyro.reset();
			strafeStraightInchesInit = false;
		}

		// If we have traveled past the distance requested, then stop.
		if (this.getEncoderDistanceAverage(WheelGroups.ALL) > inches)
		{
			strafeStraightInchesInit = true;
			this.getTransmission().stop();
			return true;
		}
		// Run the rotation in a proportional loop based on the gyro.
		this.mecanumTransmission.driveRaw(speed, Math.toRadians(directionDegrees),
				-(gyro.getAngle() * strafeStraightScalar));

		return false;
	}

	private boolean strafeStraightInchesInit = true;

	private double strafeStraightScalar = .08;

	/**
	 * Sets the scalar for the strafeStraightInches function.
	 * @param scalar
	 * 			A scalar, in percent per degrees added to the rotation of the mecanum code.
	 */
	public void setStrafeStraightScalar(double scalar)
	{
		this.strafeStraightScalar = scalar;
	}

	/**
	 * Drives the robot in a straight line based on encoders.
	 * 
	 * This works by polling the encoders every (COLLECTION_TIME) milliseconds
	 * and then taking the difference from the last collection and using it as a
	 * ratio to multiply times the speed.
	 * 
	 * This approach allows a more dynamic correction, as it only corrects as much
	 * as
	 * it needs to.
	 * 
	 * Remember: reset the encoders before running this method.
	 * 
	 * @param speed
	 *            How fast the robot will be moving. Correction will be better with
	 *            lower percentages.
	 */
	public void driveStraight(double speed)
	{
		// Only check encoders if the right amount of time has elapsed
		// (collectionTime).
		if (System.currentTimeMillis() > driveStraightOldTime + COLLECTION_TIME)
		{
			// Reset the "timer"
			driveStraightOldTime = System.currentTimeMillis();
			// Only use the four encoders if the robot uses a four-wheel system
			if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
			{
				// Calculate how much has changed between the last collection
				// time and now
				leftChange = (leftFrontEncoder.get() + leftRearEncoder.get()) - prevEncoderValues[0];
				rightChange = (rightFrontEncoder.get() + rightRearEncoder.get()) - prevEncoderValues[1];
				// Setup the previous values for the next collection run
				prevEncoderValues[0] = leftFrontEncoder.get() + leftRearEncoder.get();
				prevEncoderValues[1] = rightFrontEncoder.get() + rightRearEncoder.get();
			} else
			{
				// Calculate how much has changed between the last collection
				// time and now
				leftChange = leftRearEncoder.get() - prevEncoderValues[0];
				rightChange = rightRearEncoder.get() - prevEncoderValues[1];
				// Setup the previous values for the next collection run
				prevEncoderValues[0] = leftRearEncoder.get();
				prevEncoderValues[1] = rightRearEncoder.get();
			}
		}
		// Changes how much the robot corrects by how off course it is. The
		// more off course, the more it will attempt to correct.
		this.getTransmission().driveRaw(speed * ((double) rightChange / leftChange),
				speed * ((double) leftChange / rightChange));

	}

	private int leftChange = 1, rightChange = 1;

	private int[] prevEncoderValues =
	{ 1, 1 };
	// Preset to 1 to avoid divide by zero errors.

	// Used for calculating how much time has passed for driveStraight
	private long driveStraightOldTime = 0;

	/**
	 * Turns the robot to a certain angle using the robot's turning circle to find
	 * the arc-length.
	 * 
	 * @param angle
	 *            How far the robot should turn. Negative angle turns left, positive
	 *            turns right. (In Degrees)
	 * @param speed
	 *            How fast the robot should turn (0 to 1.0)
	 * @return Whether or not the robot has finished turning
	 */
	public boolean turnDegrees(int angle, double speed)
	{
		// Only reset the encoders on the method's first start.
		if (turnDegreesInit == true)
		{
			this.resetEncoders();
			turnDegreesInit = false;
		}

		// Tests whether any encoder has driven the arc-length of the angle
		// (angle x radius)// took out +15 on Nov 4
		if (this.transmissionType == TransmissionType.MECANUM || this.transmissionType == TransmissionType.TANK)
		{
			// Only check 4 encoders if we have a four wheel drive system
			if (this.getEncoderDistanceAverage(WheelGroups.ALL) > Math.toRadians(Math.abs(angle)) * TURNING_RADIUS)
			{
				// We have finished turning!
				this.getTransmission().stop();
				turnDegreesInit = true;
				return true;
			}
		} else
		{
			// Only check 2 encoders if we have a two wheel drive system
			if (this.getEncoderDistanceAverage(WheelGroups.REAR) > Math.toRadians(Math.abs(angle)) * TURNING_RADIUS)
			{
				// We have finished turning!
				this.getTransmission().stop();
				turnDegreesInit = true;
				return true;
			}
		}

		// Change which way the robot turns based on whether the angle is
		// positive or negative
		if (angle < 0)
		{
			this.getTransmission().driveRaw(-speed, speed);
		} else
		{
			this.getTransmission().driveRaw(speed, -speed);
		}

		return false;
	}

	/**
	 * Turns the robot based on values obtained from a gyroscopic sensor.
	 * @param angle
	 * 			At what angle we should turn, in degrees. Negative is left, positive is right.
	 * @param speed
	 * 			How fast we should turn, in decimal percentage (0.0 to  1.0)
	 * @return
	 * 			Whether or not we have finished turning.
	 */
	public boolean turnDegreesGyro(int angle, double speed)
	{
		// Reset the gyro on first start
		if (turnDegreesGyroInit)
		{
			this.gyro.reset();
			turnDegreesGyroInit = false;
		}

		// If we have traveled the number of degrees in any direction, stop.
		if (Math.abs(gyro.getAngle()) > Math.abs(angle))
		{
			this.getTransmission().stop();
			turnDegreesGyroInit = true;
			return true;
		}

		// Turn the robot based on whether we are going left or right.
		if (angle < 0)
		{
			this.getTransmission().driveRaw(-speed, speed);
		} else
		{
			this.getTransmission().driveRaw(speed, -speed);
		}

		return false;
	}

	private boolean turnDegreesGyroInit = true;

	// variable to determine if it is the first time running a method
	private boolean turnDegreesInit = true;

	// ================GAME SPECIFIC FUNCTIONS================
	/*
	 * Driving functions that change from game to game, such as using the camera
	 * to score, etc.
	 */

	// ================TUNABLES================

	// Number of milliseconds that will pass before collecting data on encoders
	// for driveStraight and brake
	private static final int COLLECTION_TIME = 10;

	// The change in inches per [COLLECTION_TIME] where the robot is considered
	// "stopped".
	private static final double BRAKE_DEADBAND = .1;

	// The distance from the left side wheel to the right-side wheel divided by
	// 2, in inches. Used in turnDegrees.
	// Nov 4 changed from 16 to 17
	private static final int TURNING_RADIUS = 16;
}
