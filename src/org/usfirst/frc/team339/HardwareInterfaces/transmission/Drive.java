package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.TransmissionType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;

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

	private GyroBase gyro = null;

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
			Encoder leftRearEncoder, Encoder rightRearEncoder, GyroBase gyro)
	{
		this.transmissionType = transmission.getType();
		this.leftFrontEncoder = leftFrontEncoder;
		this.rightFrontEncoder = rightFrontEncoder;
		this.leftRearEncoder = leftRearEncoder;
		this.rightRearEncoder = rightRearEncoder;
		this.gyro = gyro;

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
	public Drive(TransmissionBase transmission, Encoder leftEncoder, Encoder rightEncoder, GyroBase gyro)
	{
		this.transmissionType = transmission.getType();
		this.leftRearEncoder = leftEncoder;
		this.rightRearEncoder = rightEncoder;

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
			System.out.println("There was an error setting up the DRIVE class. "
					+ "Please check the declaration for a valid transmission object.");
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
		// based on what transmission type we're using, print out the object
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

	/**
	 * return Gyro to the caller
	 * 
	 * @return - this class is Gyro
	 */
	public GyroBase getGyro()
	{
		return (this.gyro);
	}

	/**
	 * 
	 * @param newGyro
	 *            - Gyro for drive to use
	 * @return - this class is Gyro
	 */

	public Gyro setGyro(GyroBase newGyro)
	{
		return (this.gyro = newGyro);
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
		 * if two wheel drive then one from each side (will be called rear of that side)
		 * if in tank or mecanum, all four wheels combined
		 */
		ALL,
		/**
		 * Both front and back on the left side
		 */
		LEFT_SIDE,
		/**
		 * Both front and back on the right side
		 */
		RIGHT_SIDE
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
			if (leftFrontEncoder != null)
				leftFrontEncoder.setDistancePerPulse(value);

			if (leftRearEncoder != null)
				leftRearEncoder.setDistancePerPulse(value);

			if (rightFrontEncoder != null)
				rightFrontEncoder.setDistancePerPulse(value);

			if (rightRearEncoder != null)
				rightRearEncoder.setDistancePerPulse(value);
			break;
		case LEFT_FRONT:
			// set distance per pulse to the left front encoder
			if (leftFrontEncoder != null)
				leftFrontEncoder.setDistancePerPulse(value);
			break;
		case RIGHT_FRONT:
			// set distance per pulse to the right front encoder
			if (rightFrontEncoder != null)
				rightFrontEncoder.setDistancePerPulse(value);
			break;
		case LEFT_REAR:
			// set distance per pulse to the left rear encoder
			if (leftRearEncoder != null)
				leftRearEncoder.setDistancePerPulse(value);
			break;
		case RIGHT_REAR:
			// set distance per pulse to the right rear encoder
			if (rightRearEncoder != null)
				rightRearEncoder.setDistancePerPulse(value);
			break;
		default:
			break;
		}
	}

	/**
	 * Sets all the encoder's stored pulses back to zero.
	 */
	public void resetEncoders()
	{
		if (leftFrontEncoder != null)
			leftFrontEncoder.reset();

		if (leftRearEncoder != null)
			leftRearEncoder.reset();

		if (rightFrontEncoder != null)
			rightFrontEncoder.reset();

		if (rightRearEncoder != null)
			rightRearEncoder.reset();
	}

	/**
	 * Gets the averages of certain wheel groups. All values are the absolute value
	 * to stop negative numbers from affecting the average.
	 * 
	 * @param encoderGroup
	 * @return
	 */
	public double getEncoderDistanceAverage(WheelGroups encoderGroup)
	{

		if (transmissionType != TransmissionType.TRACTION)
			// if we are not using traction drive then get averaged
			// distance of all the encoders
			switch (encoderGroup)
			{
			case ALL:
				// averages all the drive encoders, the two sides if
				// we're using two wheel drive and all four if mecanum or tank
				if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
				{
					return (Math.abs(leftFrontEncoder.getDistance()) + Math.abs(rightFrontEncoder.getDistance())
							+ Math.abs(leftRearEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 4.0;
				} else if (transmissionType == TransmissionType.TRACTION)
				{
					return (Math.abs(leftRearEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 2.0;
				}
				break;
			case LEFT_SIDE:
				// averages left side encoders
				return (Math.abs(leftFrontEncoder.getDistance()) + Math.abs(leftRearEncoder.getDistance())) / 2.0;
			case RIGHT_SIDE:
				// averages right side encoders
				return (Math.abs(rightFrontEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 2.0;
			default:
				return 0.0;
			}
		// If two wheel drive only
		return (Math.abs(leftRearEncoder.getDistance()) + Math.abs(rightRearEncoder.getDistance())) / 2.0;
	}

	/**
	 * Gets how many ticks is on each motor controller and adds the
	 * two of a side if multiple to get the total ticks per side
	 * 
	 * @param encoder
	 *            Which encoder position to get.
	 * @return
	 *         Number of Ticks
	 */
	public int getEncoderTicks(TransmissionBase.MotorPosition encoder)
	{
		switch (encoder)
		{
		// Returns the left front encoder if it exists. If it doesn't,
		// return left Rear encoder.
		case LEFT_FRONT:
			if (leftFrontEncoder != null)
				return leftFrontEncoder.get();

			// left Rear Encoder
		case LEFT:
		case LEFT_REAR:
			return leftRearEncoder.get();

		// Returns the right front encoder if it exists. If it doesn't,
		// return the right rear encoder.
		case RIGHT_FRONT:
			if (rightFrontEncoder != null)
				return rightFrontEncoder.get();

			// Right rear encoder
		case RIGHT:
		case RIGHT_REAR:
			return rightRearEncoder.get();

		// returns 0 to show default case was run
		default:
			return 0;
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
	public boolean isAnyEncoderLargerThan(double length)
	{
		// if using mecanum or tank return true if any of the encoders are
		// greater than the parameter, return true else return false
		if (transmissionType == TransmissionType.MECANUM || transmissionType == TransmissionType.TANK)
			return (Math.abs(leftFrontEncoder.getDistance()) > length
					|| Math.abs(rightFrontEncoder.getDistance()) > length
					|| Math.abs(leftRearEncoder.getDistance()) > length
					|| Math.abs(rightRearEncoder.getDistance()) > length);
		// else return true if either side is greater than the parameter
		return (Math.abs(leftRearEncoder.getDistance()) > length || Math.abs(rightRearEncoder.getDistance()) > length);

	}

	/**
	 * @return How many degrees the robot has turned in place since the encoders
	 *         were reset.
	 */
	public double getEncoderDegreesTurned()
	{
		return Math.toDegrees(getEncoderDistanceAverage(WheelGroups.ALL) / turningRadius);
	}

	/**
	 * Determines how many inches an encoder must read to have completed a turn
	 * @param degrees How many degrees the robot should turn
	 * @param pivot Whether or not the robot is pivoting on a wheel: Effectively doubles the turning radius
	 * @return The calculated value in inches.
	 */
	public double degreesToEncoderInches(double degrees, boolean pivot)
	{
		if (pivot == false)
			return turningRadius * Math.toRadians(Math.abs(degrees));

		return (turningRadius * 2) * Math.toRadians(Math.abs(degrees));
	}

	/**
	 * @param encoder Which encoder should be returned. If encoder is not passed into the Drive class, it will be returned as null.
	 * @return The encoder attached to the respective wheel
	 */
	public Encoder getEncoder(MotorPosition encoder)
	{
		switch (encoder)
		{
		case LEFT:
		case LEFT_REAR:
			return this.leftRearEncoder;
		case RIGHT:
		case RIGHT_REAR:
			return this.rightRearEncoder;
		case LEFT_FRONT:
			return this.leftFrontEncoder;
		case RIGHT_FRONT:
			return this.rightFrontEncoder;
		default:
			return null;
		}
	}

	// ================ DRIVE METHODS ================

	/**
	 * Resets the Drive class's functions, in case they were cut short.
	 */
	public void reset()
	{
		// sets to true
		this.driveInchesInit = true;
		this.driveStraightInchesInit = true;
		this.turnDegreesInit = true;
		this.turnDegreesGyroInit = true;
	}

	/**
	 * Enum for deciding whether we're braking after driving, or turning.
	 * 
	 * @author Ryan McGee
	 *
	 */
	public static enum BrakeType
	{
		AFTER_DRIVE, AFTER_TURN
	}

	/**
	 * Expected distance that it will take to stop during brake()
	 * 
	 * @return
	 *         expected distance during brake()
	 */
	public double getBrakeStoppingDistance()
	{
		// return distance required to brake
		return (this.distanceRequiredToBrake);
	} // end getBrakeStoppingDistance()

	/**
	 * Store the expected distance that it will take to stop during brake()
	 * 
	 * @param
	 *            brakeStoppingDistance
	 *            - new stopping distance during braking
	 * @return
	 *         new stored distance during brake()
	 */
	public double setBrakeStoppingDistance(double brakeStoppingDistance)
	{
		// sets braking distance and returns it
		return (this.distanceRequiredToBrake = brakeStoppingDistance);
	} // end setBrakeStoppingDistance()

	/**
	 * Stops the robot suddenly, to prevent drifting during autonomous functions,
	 * and increase the precision.
	 * 
	 * @param type
	 *            TODO
	 * 
	 * @return
	 *         Whether or not the robot has stopped moving.
	 */
	public boolean brake(BrakeType type)
	{
		// prints out calling brake
		// System.out.println("Calling Brake");

		// sets deadband and power to zero
		int deadband = 0;
		double power = 0;

		// if BrakeType is AFTER_DRIVE then set deadband to brakeDriveDeadband
		// and power to brakeDrivePower
		if (type == BrakeType.AFTER_DRIVE)
		{
			deadband = brakeDriveDeadband;
			power = brakeDrivePower;
		}
		// if Braketype is AFTER_TURN set the deadband to brakeTurnDeadband
		// and set power to brakeTurnPower
		else if (type == BrakeType.AFTER_TURN)
		{
			deadband = brakeTurnDeadband;
			power = brakeTurnPower;
		}

		if (System.currentTimeMillis() - previousBrakeTime > INIT_TIMEOUT)
		{
			prevEncoderValues = new int[4];

			// Get the direction of the motor values on the first start.
			brakeMotorDirection[0] = (int) Math.signum(leftRearEncoder.getRate());
			brakeMotorDirection[1] = (int) Math.signum(rightRearEncoder.getRate());
			// If it's not a 2 wheel drive, get the direction of the other 2
			// wheels.
			if (transmissionType != TransmissionType.TRACTION)
			{
				brakeMotorDirection[2] = (int) Math.signum(leftFrontEncoder.getRate());
				brakeMotorDirection[3] = (int) Math.signum(rightFrontEncoder.getRate());
			}
		}

		int[] brakeDeltas = new int[4];
		// sets values of brakeDelta array to the change in encoder ticks
		// between the current value and the brakePrevEncoderVals
		// in the order left rear, right rear, left front, right front
		brakeDeltas[0] = getEncoderTicks(MotorPosition.LEFT_REAR) - brakePrevEncoderVals[0];
		brakeDeltas[1] = getEncoderTicks(MotorPosition.RIGHT_REAR) - brakePrevEncoderVals[1];
		brakeDeltas[2] = getEncoderTicks(MotorPosition.LEFT_FRONT) - brakePrevEncoderVals[2];
		brakeDeltas[3] = getEncoderTicks(MotorPosition.RIGHT_FRONT) - brakePrevEncoderVals[3];

		// sets prev encoder tick values to the current tick vals
		brakePrevEncoderVals[0] = getEncoderTicks(MotorPosition.LEFT_REAR);
		brakePrevEncoderVals[1] = getEncoderTicks(MotorPosition.RIGHT_REAR);
		brakePrevEncoderVals[2] = getEncoderTicks(MotorPosition.LEFT_FRONT);
		brakePrevEncoderVals[3] = getEncoderTicks(MotorPosition.RIGHT_FRONT);

		// prints out all the brakeDelta array values
		// for (int i = 0; i < brakeDeltas.length; i++)
		// System.out.println("Delta " + i + ": " + brakeDeltas[i]);

		// prints out all the brakeMotorDirection array values
		// for (int i = 0; i < brakeMotorDirection.length; i++)
		// System.out.println("Power " + i + ": "
		// + brakeMotorDirection[i] * -brakeDrivePower);

		// See if the motors are past the deadband
		if (brakeMotorDirection[0] * brakeDeltas[0] < deadband && brakeMotorDirection[1] * brakeDeltas[1] < deadband
				&& ((transmissionType == TransmissionType.TRACTION)
						|| brakeMotorDirection[2] * brakeDeltas[2] < deadband
						|| brakeMotorDirection[3] * brakeDeltas[3] < deadband))
		{
			// Increase the iteration
			currentBrakeIteration++;
		} else
		{
			// Reset the iteration. We want x times ~in a row~.
			currentBrakeIteration = 0;
		}

		// If we have been within the deadband for x times, return true.
		if (currentBrakeIteration >= totalBrakeIterations)
		{
			currentBrakeIteration = 0;
			getTransmission().stop();
			return true;
		}

		// Set the rear wheels
		getTransmission().getSpeedController(MotorPosition.LEFT_REAR).set(-brakeMotorDirection[0] * power);
		getTransmission().getSpeedController(MotorPosition.RIGHT_REAR).set(-brakeMotorDirection[1] * power);
		// Set the front wheels if it's the right kind of drive
		if (transmissionType != TransmissionType.TRACTION)
		{
			getTransmission().getSpeedController(MotorPosition.LEFT_FRONT).set(-brakeMotorDirection[2] * power);
			getTransmission().getSpeedController(MotorPosition.RIGHT_FRONT).set(-brakeMotorDirection[3] * power);
		}
		// END SET MOTORS
		this.previousBrakeTime = System.currentTimeMillis();
		return false;
	}

	private long previousBrakeTime = 0; // milliseconds

	private int[] brakeMotorDirection = new int[4];

	private int brakeDriveDeadband = 20; // ticks

	private int brakeTurnDeadband = 12;// ticks

	private int[] brakePrevEncoderVals = new int[4];

	private double brakeDrivePower = .2;

	private double brakeTurnPower = 1.0;

	private int currentBrakeIteration = 0;

	private int totalBrakeIterations = 3;

	/**
	 * Sets the deadband for brake()... how close to stopped we are.
	 * 
	 * @param ticks
	 *            Ticks on the encoder, not distance.
	 * @param type
	 *            What kind of turn this is being called after
	 */
	public void setBrakeDeadband(int ticks, BrakeType type)
	{
		switch (type)
		{
		case AFTER_DRIVE:
			this.brakeDriveDeadband = ticks;
			break;
		case AFTER_TURN:
			this.brakeTurnDeadband = ticks;
			break;
		default:
			break;
		}

	}

	/**
	 * Sets how much the robot should send to the motors while braking
	 * 
	 * @param power
	 *            percentage (0.0 to 1.0)
	 * @param type
	 *            What kind of turn this is being called after
	 */
	public void setBrakePower(double power, BrakeType type)
	{
		switch (type)
		{
		case AFTER_DRIVE:
			this.brakeDrivePower = power;
			break;
		case AFTER_TURN:
			this.brakeTurnPower = power;
			break;
		default:
			break;
		}
	}

	/**
	 * 
	 * 
	 * @param iterations
	 */
	public void setBrakeIterations(int iterations)
	{
		this.totalBrakeIterations = iterations;
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

		// Test if ANY encoder is past the distance. stop if there is
		if (this.isAnyEncoderLargerThan(distance) == true)
		{
			this.driveInchesInit = true;
			this.getTransmission().stop();
			return true;
		}

		// sets transmission speed to the input
		this.getTransmission().drive(speed, speed);
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
	 * @param accelerate TODO
	 * @return Whether or not the robot has finished traveling that given distance.
	 */
	public boolean driveStraightInches(double distance, double speed, boolean accelerate)
	{
		// Runs once when the method runs the first time, and does not run again
		// until after the method returns true.
		if (driveStraightInchesInit == true)
		{
			this.resetEncoders();
			driveStraightInchesInit = false;
		}

		// Check all encoders to see if they've reached the distance
		if (this.isAnyEncoderLargerThan(Math.abs(distance)) == true)
		{
			this.getTransmission().stop();
			driveStraightInchesInit = true;
			return true;
		}

		// Drive straight if we have not reached the distance
		this.driveStraight(speed, accelerate);

		return false;
	}

	private boolean driveStraightInchesInit = true;

	/**
	 * Strafe to a target using a mecanum transmission, and a gyro for
	 * stabilization.
	 * This will NOT be accurate because of mecanum's slippery properties.
	 * 
	 * @param inches
	 *            How far we should travel
	 * @param speed
	 *            How fast we should travel, in decimal percentage (0.0 to 1.0)
	 * @param directionDegrees
	 *            In which direction we should travel, 0 being forwards, -90 for
	 *            left and 90 for right.
	 * @return
	 *         Whether or not we have finished strafing.
	 */
	public boolean strafeStraightInches(int inches, double speed, int directionDegrees)
	{
		// Wrong transmission type! we cant strafe if we dont have the right
		// transmission
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
		this.mecanumTransmission.drive(speed, Math.toRadians(directionDegrees),
				-(gyro.getAngle() * strafeStraightScalar));

		return false;
	}

	private boolean strafeStraightInchesInit = true;

	private double strafeStraightScalar = .08;

	/**
	 * Sets the scalar for the strafeStraightInches function.
	 * 
	 * @param scalar
	 *            A scalar, in percent per degrees added to the rotation of the
	 *            mecanum code.
	 */
	public void setStrafeStraightScalar(double scalar)
	{
		// sets strafe straight scalar to thr input
		this.strafeStraightScalar = scalar;
	}

	/**
	 * Accelerates the robot to a certain speed, then continue at the speed input
	 * into the method. Allows for accelerate and driving straight directly
	 * afterwards.
	 * 
	 * The equation is: deltaTime / (timeRequested) * speed. The code will make sure
	 * that deltaTime / timeRequested does not return outside -1 to 1.
	 * 
	 * @param leftSpeed
	 *            The target speed for the left drive motors
	 * @param rightSpeed
	 *            The target speed for the right drive motors
	 * @param time
	 *            The time period accelerated over, in seconds. When the time
	 *            reaches this
	 *            number, it will be running full speed.
	 * 
	 * @return True if we are done accelerating. It WILL continue driving after
	 *         acceleration is done, at the input speed.
	 */
	public boolean accelerateTo(double leftSpeed, double rightSpeed, double time)
	{
		// Avoid a divideByZero error.
		if (time <= 0)
		{
			this.getTransmission().drive(leftSpeed, rightSpeed);
			return true;
		}

		// If we timeout, then reset all the accelerate
		if (System.currentTimeMillis() - lastAccelerateTime > INIT_TIMEOUT || accelerateInit == true)
		{
			lastAccelerateTime = System.currentTimeMillis();
			accelMotorPower = accelStartingSpeed;
			accelerateInit = false;
		}

		// main acceleration maths
		double deltaSeconds = (System.currentTimeMillis() - lastAccelerateTime) / 1000.0;
		accelMotorPower += deltaSeconds / time;

		// Drive the robot based on the times and speeds
		this.getTransmission().drive(leftSpeed * inRange(accelMotorPower), rightSpeed * inRange(accelMotorPower));

		// Reset the "timer"
		lastAccelerateTime = System.currentTimeMillis();

		if (accelMotorPower > 1.0)
		{
			return true;
		}
		// We are not done accelerating
		return false;
	}

	private double accelMotorPower = 0;// Power sent to each motor

	private double accelStartingSpeed = .15;

	private long lastAccelerateTime = 0; // Milliseconds

	private double defaultAcceleration = .8;// Seconds

	private boolean accelerateInit = true;

	/**
	 * Sets the initial speed of the accelerateTo motors
	 * 
	 * @param value
	 *            Positive percentage / motor value
	 */
	public void setAccelStartingSpeed(double value)
	{
		// sets accelStartingSpeed to the input value
		this.accelStartingSpeed = value;
	}

	/**
	 * Resets the accelerate function, if the robot changes direction too fast.
	 */
	public void resetAccelerate()
	{
		this.accelerateInit = true;
	}

	/**
	 * Sets the default acceleration for driveStraight
	 * 
	 * @param value
	 *            The acceleration period, in seconds
	 */
	public void setDefaultAcceleration(double value)
	{
		// sets default acceleration to the input value
		this.defaultAcceleration = .8;
	}

	/**
	 * Drives the robot in a straight line, correcting based on values gotten from
	 * encoders.
	 * 
	 * 
	 * @param speed
	 *            How fast the robot should be moving, and in which direction.
	 * @param acceleration
	 *            Whether or not the robot should accelerate to the requested speed.
	 */
	public void driveStraight(double speed, boolean acceleration)
	{
		// "Reset" the encoders (will not mess with driveInches or such)
		if (System.currentTimeMillis() - driveStraightLastTime > INIT_TIMEOUT)
		{
			// this.prevEncoderValues[0] = getEncoderTicks(MotorPosition.LEFT);
			// this.prevEncoderValues[1] = getEncoderTicks(
			// MotorPosition.RIGHT);
			this.gyro.reset();
		}

		// Left change since start minus right change since start
		// int delta = (getEncoderTicks(MotorPosition.LEFT)
		// - prevEncoderValues[0])
		// - (getEncoderTicks(MotorPosition.RIGHT)
		// - prevEncoderValues[1]);

		// If left is greater than right, add more to right & subtract from
		// left.
		// If right is greater than left, add more to left & subtract from
		// right.
		double leftSpeed = speed - (Math.signum(gyro.getAngle()) * driveStraightConstant);
		double rightSpeed = speed + (Math.signum(gyro.getAngle()) * driveStraightConstant);

		if (acceleration)
			this.accelerateTo(leftSpeed, rightSpeed, getDefaultAcceleration());
		else
			this.accelerateTo(leftSpeed, rightSpeed, 0);
		// Reset the "timer" to know when to "reset" the encoders for this
		// method.
		driveStraightLastTime = System.currentTimeMillis();
	}

	private long driveStraightLastTime = 0;

	/**
	 * Drives the robot in a straight line based on encoders.
	 * 
	 * This works by polling the encoders every (COLLECTION_TIME) milliseconds
	 * and then taking the difference from the last collection and using it as a
	 * ratio to multiply times the speed.
	 * 
	 * This approach allows a more dynamic correction, as it only corrects as much
	 * as it needs to.
	 * 
	 * Remember: reset the encoders before running this method.
	 * 
	 * @deprecated Method relies on change between cycles on the rio, not absolute
	 *             positioning. Use the improved driveStraight instead.
	 * 
	 * @param speed
	 *            How fast the robot will be moving. Correction will be better with
	 *            lower percentages.
	 * @param accelerate
	 *            TODO
	 * @param acceleration
	 *            TODO
	 */
	public void driveStraight_old(double speed, boolean accelerate)
	{
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

		double delta = leftChange - rightChange;
		double leftMotorVal = speed, rightMotorVal = speed;

		leftMotorVal -= Math.signum(delta) * driveStraightConstant;
		rightMotorVal += Math.signum(delta) * driveStraightConstant;

		// Changes how much the robot corrects by how off course it is. The
		// more off course, the more it will attempt to correct.

		if (accelerate)
			this.accelerateTo(leftMotorVal, rightMotorVal, getDefaultAcceleration());
		else
			this.getTransmission().drive(leftMotorVal, rightMotorVal);

	}

	// How much should be subtracted from the left or right side while driving
	// straight
	private double driveStraightConstant = .1;

	private double leftChange = 0, rightChange = 0;

	private int[] prevEncoderValues =
	{ 0, 0 };

	/**
	 * Sets how much the robot should correct while driving straight.
	 * 
	 * @param value
	 *            Percentage (0.0 to 1.0)
	 */
	public void setDriveStraightConstant(double value)
	{
		// sets Drive Straight Constant to the input
		this.driveStraightConstant = value;
	}

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

		// Only check 4 encoders if we have a four wheel drive system
		// if two wheel drive checks, one of each side(called rear of that side)
		if (this.getEncoderDistanceAverage(WheelGroups.ALL) > Math.toRadians(Math.abs(angle)) * turningRadius)
		{
			// We have finished turning!
			this.getTransmission().stop();
			turnDegreesInit = true;
			return true;
		}

		// Change which way the robot turns based on whether the angle is
		// positive or negative
		if (angle < 0)
		{
			this.getTransmission().drive(-speed, speed);
		} else
		{
			this.getTransmission().drive(speed, -speed);
		}

		return false;
	}

	/**
	 * Turns the robot based on values obtained from a gyroscopic sensor.
	 * 
	 * @param angle
	 *            At what angle we should turn, in degrees. Negative is left,
	 *            positive is right.
	 * @param speed
	 *            How fast we should turn, in decimal percentage (0.0 to 1.0)
	 * @return
	 *         Whether or not we have finished turning.
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
		if (Math.abs(gyro.getAngle()) >= Math.abs(angle))
		{
			this.getTransmission().stop();
			turnDegreesGyroInit = true;
			return true;
		}

		// Turn the robot based on whether we are going left or right.
		if (angle < 0)
		{
			this.getTransmission().drive(-speed, speed);
		} else
		{
			this.getTransmission().drive(speed, -speed);
		}

		return false;
	}

	private boolean turnDegreesGyroInit = true;

	// variable to determine if it is the first time running a method
	private boolean turnDegreesInit = true;

	/**
	 * Turns the robot by pivoting either on one wheel or the other. Only useful for
	 * 2 wheel drive systems.
	 * 
	 * @param degrees
	 *            number of degrees the robot should turn. Negative for
	 *            counter-clockwise, positive for clockwise.
	 * @param power
	 *            How fast the robot should be turning, in percentage (0.0 to 1.0)
	 * @return
	 *         Whether or not the robot has finished turning.
	 */
	public boolean pivotTurnDegrees(int degrees, double power)
	{
		// Reset the encoders on the first start only
		if (pivotTurnDegreesInit == true)
		{
			this.gyro.reset();
			pivotTurnDegreesInit = false;
		}

		// Turning clockwise
		if (degrees > 0)
		{
			// If the left side has reached it's calculated arc length, stop.
			if (Math.abs(gyro.getAngle()) > Math.abs(degrees) - this.gyroFudgeFactor)
			{
				this.getTransmission().stop();
				pivotTurnDegreesInit = true;
				return true;
			}
			this.getTransmission().drive(power, -pivotDegreesStationaryPercentage);
		}
		// Turning counter-clockwise
		else
		{
			// If the right side has reached it's calculated arc length, stop.
			if (Math.abs(gyro.getAngle()) > Math.abs(degrees) - this.gyroFudgeFactor)
			{
				this.getTransmission().stop();
				pivotTurnDegreesInit = true;
				return true;
			}
			this.getTransmission().drive(-pivotDegreesStationaryPercentage, power);
		}
		return false;
	}

	private boolean pivotTurnDegreesInit = true;

	private double pivotDegreesStationaryPercentage = .1;

	/**
	 * Turns the robot using the gyro, and slows down after passing
	 * "turnDegreesTriggerStage" degrees from the requested degrees.
	 * 
	 * @param degrees
	 *            How much the robot should turn, in degrees. Positive for
	 *            clockwise, negative for counter-clockwise.
	 * @param power
	 *            How fast the robot should turn, in percentage (0.0 to 1.0)
	 * @return
	 *         Whether or not the robot has finished turning.
	 */
	public boolean turnDegrees2Stage(int degrees, double power)
	{
		if (turnDegrees2StageInit == true)
		{
			this.gyro.reset();
			turnDegrees2StageInit = false;
		}

		// If we have turned (degrees) at all (left or right, just in case),
		// return
		// true.
		if (Math.abs(gyro.getAngle()) > Math.abs(degrees) - gyroFudgeFactor)
		{
			this.getTransmission().stop();
			turnDegrees2StageInit = true;
			return true;
		}

		// 2nd stage run slow
		if (Math.abs(this.gyro.getAngle()) > Math.abs(degrees) - turnDegreesTriggerStage)
		{
			this.getTransmission().drive(Math.signum(degrees) * turnDegrees2ndStagePower,
					-Math.signum(degrees) * turnDegrees2ndStagePower);
		}
		// 1st stage run (power)
		else
		{
			this.getTransmission().drive(Math.signum(degrees) * power, -Math.signum(degrees) * power);
		}

		return false;
	}

	private double gyroFudgeFactor = 3; // degrees

	/**
	 * Sets the distance from the wheel to the turning center point.
	 * 
	 * @param radius
	 *            Distance, in inches.
	 */
	public void setTurningRadius(double radius)
	{
		turningRadius = radius;
	}

	/**
	 * @return the defaultAcceleration, in seconds
	 */
	public double getDefaultAcceleration()
	{
		return defaultAcceleration;
	}

	private boolean turnDegrees2StageInit = true;

	private double turnDegreesTriggerStage = 40;// Degrees

	private double turnDegrees2ndStagePower = .19;

	// ---------------------------------
	// THis is the distance we expect to move
	// during a brake call (in inches)
	// ---------------------------------
	private double distanceRequiredToBrake = 3.0;

	// ================TUNABLES================

	// Number of milliseconds that will pass before collecting data on encoders
	// for driveStraight and brake
	private static final int COLLECTION_TIME = 100;

	// The distance from the left side wheel to the right-side wheel divided by
	// 2, in inches. Used in turnDegrees.
	// Nov 4 changed from 16 to 17
	private static double turningRadius = 11 - .35;

	private static final int INIT_TIMEOUT = 300;// Milliseconds until the
												// initialization should reset.
}
