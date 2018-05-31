package org.usfirst.frc.team339.Utils.drive;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyEncoder;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.MecanumTransmission;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.SwerveTransmission;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TankTransmission;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.TransmissionType;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * The class that controls autonomous driving functions or driver-assisting
 * functions based on sensors.
 * 
 * @author Ryan McGee
 * @written 7/26/2017
 */
public class Drive
{

	/**
	 * Creates the Drive object. If a sensor listed is not used (except for
	 * encoders), set it to null. Setup for Traction drive (only 2 motors/encoders)
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
	 *            A sensor that is used to measure rotation
	 */
	public Drive(TransmissionBase transmission, KilroyEncoder leftEncoder, KilroyEncoder rightEncoder, GyroBase gyro)
	{
		this.transmissionType = transmission.getType();
		this.transmission = transmission;
		this.encoders = new KilroyEncoder[2];
		this.encoders[0] = leftEncoder;
		this.encoders[1] = rightEncoder;

		this.gyro = gyro;

	}

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
	 *            A sensor that is used to measure rotation.
	 * @param visionProcessor
	 *            The camera's vision processing code, as a sensor.
	 */
	public Drive(TransmissionBase transmission, KilroyEncoder leftRearEncoder, KilroyEncoder rightRearEncoder,
			KilroyEncoder leftFrontEncoder, KilroyEncoder rightFrontEncoder, GyroBase gyro)
	{
		this.transmissionType = transmission.getType();
		this.transmission = transmission;
		this.encoders = new KilroyEncoder[4];
		this.encoders[0] = leftRearEncoder;
		this.encoders[1] = rightRearEncoder;
		this.encoders[2] = leftFrontEncoder;
		this.encoders[3] = rightFrontEncoder;
		this.gyro = gyro;

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
	 *            reaches this number, it will be running full speed.
	 * 
	 * @return True if we are done accelerating. It WILL continue driving after
	 *         acceleration is done, at the input speed.
	 */
	public boolean accelerateTo(double leftSpeed, double rightSpeed, double time)
	{
		// Avoid a divideByZero error.
		if (time <= 0)
		{
			this.transmission.driveRaw(leftSpeed, rightSpeed);
			return true;
		}

		// If we timeout, then reset all the accelerate
		if (System.currentTimeMillis() - lastAccelerateTime > INIT_TIMEOUT)
		{
			lastAccelerateTime = System.currentTimeMillis();
			accelMotorPower = accelStartingSpeed;
		}

		// main acceleration maths
		double deltaSeconds = (System.currentTimeMillis() - lastAccelerateTime) / 1000.0;
		accelMotorPower += deltaSeconds / time;

		// Drive the robot based on the times and speeds
		this.transmission.driveRaw(leftSpeed * inRange(accelMotorPower), rightSpeed * inRange(accelMotorPower));

		// Reset the "timer"
		lastAccelerateTime = System.currentTimeMillis();

		if (accelMotorPower > 1.0)
		{
			return true;
		}
		// We are not done accelerating
		return false;
	}

	/**
	 * Drive the robot in an arc around a point defined as 'radius' inches away from
	 * the center of the robot. This calculates the percentage based on the ratio of
	 * the circumference of the circle created by the left wheel and right wheel.
	 * 
	 * This can be used as a less accurate, but MUCH faster way of turning the
	 * robot, rather than turning in place. Related to pivotTurn.
	 * 
	 * @param speed
	 *            How fast the robot should travel, in percentage. Positive is
	 *            forwards, negative is backwards.
	 * @param radius
	 *            The distance between the center point of the arc and the center of
	 *            the bot. Positive is for a right turn, negative is for a left
	 *            turn.
	 * @param arcLength
	 *            How far the robot should travel along that arc length
	 * @param acceleration
	 *            Over how much time, in seconds, the robot should accelerate over.
	 * @return Whether or not the robot has driven it's arc length.
	 */
	public boolean arc(double speed, double radius, double arcLength, double acceleration)
	{
		// Initialize by resetting the sensor.
		if (arcInit == true)
		{
			this.resetEncoders();
			arcInit = false;
		}

		// We have reached the arc length? then we are done.
		if (getEncoderDistanceAverage(MotorPosition.ALL) > arcLength)
		{
			getTransmission().stop();
			return true;
		}

		double innerCircle = 2 * (radius - turningRadius) * Math.PI;// The
																	// circumference
																	// of the
																	// smaller
																	// circle
		double outerCircle = 2 * (radius + turningRadius) * Math.PI;// The
																	// circumference
																	// of the
																	// larger
																	// circle

		// Determines which way we are turning, based on if the radius is
		// positive or
		// negative.
		double leftSide = 0, rightSide = 0;
		if (radius > 0)
		{
			leftSide = 1;
			rightSide = innerCircle / outerCircle;
		} else if (radius < 0)
		{
			rightSide = 1;
			leftSide = innerCircle / outerCircle;
		}

		this.accelerateTo(speed * leftSide, speed * rightSide, acceleration);

		return false;
	}

	/**
	 * Stops the robot suddenly, to prevent drifting during autonomous functions,
	 * and increase the precision.
	 * 
	 * @param type
	 *            What kind of brake is being used, after driving, or after turning.
	 * 
	 * @return Whether or not the robot has stopped moving.
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
			brakePrevEncoderVals = new int[4];

			// Get the direction of the motor values on the first start.
			brakeMotorDirection[0] = (int) Math.signum(encoders[0].getRate());
			brakeMotorDirection[1] = (int) Math.signum(encoders[1].getRate());
			// If it's not a 2 wheel drive, get the direction of the other 2
			// wheels.
			if (encoders.length >= 4)
			{
				brakeMotorDirection[2] = (int) Math.signum(encoders[3].getRate());
				brakeMotorDirection[3] = (int) Math.signum(encoders[4].getRate());
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
				&& ((encoders.length < 3) || brakeMotorDirection[2] * brakeDeltas[2] < deadband
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
			transmission.stop();
			return true;
		}

		// Set the rear wheels
		transmission.getSpeedController(MotorPosition.LEFT_REAR).set(-brakeMotorDirection[0] * power);
		transmission.getSpeedController(MotorPosition.RIGHT_REAR).set(-brakeMotorDirection[1] * power);
		// Set the front wheels if it's the right kind of drive
		if (encoders.length >= 4)
		{
			transmission.getSpeedController(MotorPosition.LEFT_FRONT).set(-brakeMotorDirection[2] * power);
			transmission.getSpeedController(MotorPosition.RIGHT_FRONT).set(-brakeMotorDirection[3] * power);
		}
		// END SET MOTORS
		this.previousBrakeTime = System.currentTimeMillis();
		return false;
	}

	/**
	 * Determines how many inches an encoder must read to have completed a turn
	 * 
	 * @param degrees
	 *            How many degrees the robot should turn
	 * @param pivot
	 *            Whether or not the robot is pivoting on a wheel: Effectively
	 *            doubles the turning radius
	 * @return The calculated value in inches.
	 */
	public double degreesToEncoderInches(double degrees, boolean pivot)
	{
		if (pivot == false)
			return turningRadius * Math.toRadians(Math.abs(degrees));

		return (turningRadius * 2) * Math.toRadians(Math.abs(degrees));
	}

	/**
	 * Drives the robot with the use of two joysticks, for Tank drive.
	 * 
	 * @param leftJoystick
	 * @param rightJoystick
	 */
	public void drive(Joystick leftJoystick, Joystick rightJoystick)
	{
		if (transmission instanceof TankTransmission)
			((TankTransmission) transmission).drive(leftJoystick, rightJoystick);
	}

	/**
	 * Drives the robot with the use of two values, for Tank drive.
	 * This DOES use gear ratios and joystick deadbands.
	 * @param leftVal From -1.0 (backwards) to 1.0 (forwards)
	 * @param rightVal From -1.0 (backwards) to 1.0 (forwards)
	 */
	public void drive(double leftVal, double rightVal)
	{
		if (transmission instanceof TankTransmission)
			((TankTransmission) transmission).drive(leftVal, rightVal);
	}

	/**
	 * Drives the robot with a omni-directional drive, with a single 3 axis
	 * joystick.
	 * 
	 * @param joystick
	 */
	public void drive(Joystick joystick)
	{
		if (transmission instanceof MecanumTransmission)
			((MecanumTransmission) transmission).drive(joystick);
		else if (transmission instanceof SwerveTransmission)
			((SwerveTransmission) transmission).drive(joystick);
	}

	/**
	 * Drives the robot with a omni-directional drive, with 3 separate raw values.
	 * This DOES use deadbands and gear ratios.
	 * @param magnitude Speed, from 0.0 to 1.0
	 * @param direction Angle for strafing, from -180 to 180
	 * @param rotation Speed in turns, from -1.0 (left) to 1.0 (right)
	 */
	public void drive(double magnitude, double direction, double rotation)
	{
		if (transmission instanceof MecanumTransmission)
			((MecanumTransmission) transmission).drive(magnitude, direction, rotation);
		else if (transmission instanceof SwerveTransmission)
			((SwerveTransmission) transmission).drive(magnitude, direction, rotation);
	}

	/**
	 * Drives the robot a certain distance without encoder correction. Not using
	 * correction increases reliability but decreases precision. If one encoder
	 * fails, it will instead look for other encoders for input.
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
			this.transmission.stop();
			return true;
		}

		// sets transmission speed to the input
		this.transmission.driveRaw(speed, speed);
		return false;
	}

	/**
	 * Drives the robot in a straight line, correcting based on values gotten from
	 * encoders.
	 * 
	 * @param speed
	 *            How fast the robot should be moving, and in which direction.
	 * @param acceleration
	 *            Whether or not the robot should accelerate to the requested speed.
	 * @param isUsingGyro
	 *            If true, the chosen sensor is a gyro. If false, it uses encoders.
	 */
	public void driveStraight(double speed, boolean acceleration, boolean isUsingGyro)
	{
		// "Reset" the encoders (will not mess with driveInches or such)
		if (System.currentTimeMillis() - driveStraightLastTime > INIT_TIMEOUT)
		{
			if (isUsingGyro == true)
				this.gyro.reset();
			else
				this.resetEncoders();
		}

		double leftSpeed = 0;
		double rightSpeed = 0;
		// If left is greater than right, add more to right & subtract from
		// left.
		// If right is greater than left, add more to left & subtract from
		// right.
		if (isUsingGyro == true)
		{
			leftSpeed = speed - (Math.signum(gyro.getAngle()) * driveStraightConstant);
			rightSpeed = speed + (Math.signum(gyro.getAngle()) * driveStraightConstant);
		} else
		{
			leftSpeed = speed - (Math.signum(
					getEncoderDistanceAverage(MotorPosition.LEFT) - getEncoderDistanceAverage(MotorPosition.RIGHT))
					* driveStraightConstant);
			rightSpeed = speed - (Math.signum(
					getEncoderDistanceAverage(MotorPosition.LEFT) - getEncoderDistanceAverage(MotorPosition.RIGHT))
					* driveStraightConstant);
		}

		// Only send the new power to the side lagging behind
		if (leftSpeed > rightSpeed)
		{
			rightSpeed = speed;
		} else
		{
			leftSpeed = speed;
		}

		if (acceleration)
			this.accelerateTo(leftSpeed, rightSpeed, getDefaultAcceleration());
		else
			this.accelerateTo(leftSpeed, rightSpeed, 0);
		// Reset the "timer" to know when to "reset" the encoders for this
		// method.
		driveStraightLastTime = System.currentTimeMillis();
	}

	/**
	 * Drives the robot a certain distance based on the encoder values. If the robot
	 * should go backwards, set speed to be negative instead of distance.
	 * 
	 * If the acceleration does not seem to be working, run reset() between states.
	 * 
	 * @param distance
	 *            How far the robot should go (should be greater than 0)
	 * @param speed
	 *            How fast the robot should travel
	 * @param accelerate
	 *            Whether or not the robot is accelerating
	 * @param isUsingGyro
	 *            If true, the chosen sensor is a gyro. If false, it uses encoders.
	 * @return Whether or not the robot has finished traveling that given distance.
	 */
	public boolean driveStraightInches(double distance, double speed, boolean accelerate, boolean isUsingGyro)
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
			this.transmission.stop();
			driveStraightInchesInit = true;
			return true;
		}

		// Drive straight if we have not reached the distance
		this.driveStraight(speed, accelerate, true);

		return false;
	}

	/**
	 * Expected distance that it will take to stop during brake()
	 * 
	 * @return expected distance during brake()
	 */
	public double getBrakeStoppingDistance()
	{
		// return distance required to brake
		return (this.distanceRequiredToBrake);
	} // end getBrakeStoppingDistance()

	/**
	 * @return the current software gear, starting at 0 as the slowest.
	 */
	public int getCurrentGear()
	{
		return transmission.getCurrentGear();
	}

	/**
	 * @return the current ratio that is being multiplied onto the
	 *  drive inputs, from 0.0 to 1.0
	 */
	public double getCurrentGearRatio()
	{
		return transmission.getCurrentGearRatio();
	}

	/**
	 * @return the defaultAcceleration, in seconds
	 */
	public double getDefaultAcceleration()
	{
		return defaultAcceleration;
	}

	/**
	 * @param encoder
	 *            Which encoder should be returned. If encoder is not passed into
	 *            the Drive class, it will be returned as null.
	 * @return The encoder attached to the respective wheel
	 */
	public KilroyEncoder getEncoder(MotorPosition encoder)
	{
		switch (encoder)
		{
		case LEFT:
		case LEFT_REAR:
			return this.encoders[0];
		case RIGHT:
		case RIGHT_REAR:
			return this.encoders[1];
		case LEFT_FRONT:
			if (encoders.length > 2)
				return this.encoders[2];
		case RIGHT_FRONT:
			if (encoders.length > 3)
				return this.encoders[3];
		default:
			return null;
		}
	}

	// ================ENCODER METHODS================

	/**
	 * @return How many degrees the robot has turned in place since the encoders
	 *         were reset.
	 */
	public double getEncoderDegreesTurned()
	{
		return Math.toDegrees(getEncoderDistanceAverage(MotorPosition.ALL) / turningRadius);
	}

	/**
	 * Gets the averages of certain wheel groups. If ALL is selected, then the
	 * absolute value is run to avoid issues where the average cancels them out.
	 * 
	 * @param encoderGroup
	 *            Which wheel / set to find the average of. only LEFT, RIGHT and ALL
	 *            are accepted.
	 * @return the final averaged distance
	 */
	public double getEncoderDistanceAverage(MotorPosition encoderGroup)
	{
		double added = 0;
		switch (encoderGroup)
		{
		case LEFT:
			// Average them, if necessary
			for (int i = 0; i < encoders.length; i++)
				if (i % 2 == 0)
					added += encoders[i].getDistance();
			return added / (encoders.length / 2);
		case RIGHT:
			// Average them, if necessary
			for (int i = 0; i < encoders.length; i++)
				if (i % 2 == 1)
					added += encoders[i].getDistance();
			return added / (encoders.length / 2);
		default:
		case ALL:
			for (int i = 0; i < encoders.length; i++)
				added += encoders[i].getDistance();
			return Math.abs(added) / encoders.length;
		// Absolute value, in case turning makes it near 0
		}
	}

	/**
	 * Gets how many ticks is on each motor controller and adds the two of a side if
	 * multiple to get the total ticks per side
	 * 
	 * @param encoder
	 *            Which encoder position to get.
	 * @return Number of Ticks
	 */
	public int getEncoderTicks(MotorPosition encoder)
	{
		switch (encoder)
		{
		case LEFT:
		case LEFT_REAR:
			return encoders[0].get();
		case RIGHT:
		case RIGHT_REAR:
			return encoders[1].get();
		case LEFT_FRONT:
			if (encoders.length > 2)
				return encoders[2].get();
		case RIGHT_FRONT:
			if (encoders.length > 3)
				return encoders[3].get();
			// returns 0 to show default case was run
		default:
			return 0;
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
	 * Gets the transmission object stored. ONLY use it for transmission.stop() and
	 * transmission.driveRaw()
	 * 
	 * @return The current transmission object used in the Drive class
	 */
	public TransmissionBase getTransmission()
	{
		return this.transmission;
	}

	/**
	 * Tests whether any encoder reads larger than the input length. Useful for
	 * knowing when to stop the robot.
	 * 
	 * @param length
	 *            The desired length
	 * @return True when any encoder is past length
	 */
	public boolean isAnyEncoderLargerThan(double length)
	{
		for (KilroyEncoder enc : encoders)
			if (Math.abs(enc.getDistance()) > length)
				return true;
		return false;
	}

	/**
	 * Turns the robot by pivoting on one side or another.
	 * 
	 * @param degrees
	 *            number of degrees the robot should turn. Negative for
	 *            counter-clockwise, positive for clockwise.
	 * @param power
	 *            How fast the robot should be turning, in percentage (0.0 to 1.0)
	 * @param usingGyro
	 *            If true, then the method will rely on the Gyro as it's sensor. If
	 *            false, then it will rely on encoders.
	 * @return Whether or not the robot has finished turning.
	 */
	public boolean pivotTurnDegrees(int degrees, double power, boolean usingGyro)
	{
		// Reset the encoders on the first start only
		if (pivotTurnDegreesInit == true)
		{
			if (usingGyro)
				this.gyro.reset();
			else
				this.resetEncoders();
			pivotTurnDegreesInit = false;
		}

		boolean finished = false;

		// If we are using the gyro, and the gyro has been passed in, then check
		// it.
		if (usingGyro && this.gyro != null)
		{
			if (Math.abs(gyro.getAngle()) > Math.abs(degrees) - this.gyroFudgeFactor)
				finished = true;
		}
		// If we are NOT using the gyro, use the encoders.
		else
		{
			if (degrees > 0
					&& Math.abs(getEncoderDistanceAverage(MotorPosition.LEFT)) > degreesToEncoderInches(degrees, true))
				finished = true;
			else if (Math.abs(getEncoderDistanceAverage(MotorPosition.RIGHT)) > degreesToEncoderInches(degrees, true))
				finished = true;
		}

		// We have reached the angle, so stop.
		if (finished == true)
		{
			this.transmission.stop();
			pivotTurnDegreesInit = true;
			return true;
		}

		// Turning clockwise
		if (degrees > 0)
			this.transmission.driveRaw(power, -pivotDegreesStationaryPercentage);
		// Turning counter-clockwise
		else
			this.transmission.driveRaw(-pivotDegreesStationaryPercentage, power);

		return false;
	}

	/**
	 * Resets the Drive class's functions, in case they were cut short.
	 * 
	 * This is to make sure initialization of each method works accordingly.
	 */
	public void reset()
	{
		// sets to true
		this.driveInchesInit = true;
		this.driveStraightInchesInit = true;
		this.turnDegreesInit = true;
		this.turnDegreesGyroInit = true;
		this.pivotTurnDegreesInit = true;
		this.strafeStraightInchesInit = true;
		this.turnDegrees2StageInit = true;

		this.currentBrakeIteration = 0;
		this.lastAccelerateTime = 0;
		this.previousBrakeTime = 0;
	}

	// ================ DRIVE METHODS ================

	/**
	 * Resets the accelerate function, if the robot changes direction too fast.
	 * 
	 * @deprecated use {@link #reset() reset()} instead.
	 */
	public void resetAccelerate()
	{
		this.reset();
	}

	/**
	 * Sets all the encoder's stored pulses back to zero.
	 */
	public void resetEncoders()
	{
		for (KilroyEncoder enc : encoders)
			enc.reset();
	}

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
	 * Sets all the gear ratios of the robot, from lowest to highest.
	 * This corresponds with gear 0, gear 1, gear 2, etc.
	 * @param ratios The ratios, from 0.0 to 1.0.
	 */
	public void setAllGearPercentages(double... ratios)
	{
		this.transmission.setAllGearPercentages(ratios);
	}

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
	 * 
	 * 
	 * @param iterations
	 */
	public void setBrakeIterations(int iterations)
	{
		this.totalBrakeIterations = iterations;
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
	 * Store the expected distance that it will take to stop during brake()
	 * 
	 * @param brakeStoppingDistance
	 *            - new stopping distance during braking
	 * @return new stored distance during brake()
	 */
	public double setBrakeStoppingDistance(double brakeStoppingDistance)
	{
		// sets braking distance and returns it
		return (this.distanceRequiredToBrake = brakeStoppingDistance);
	} // end setBrakeStoppingDistance()

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
	 * Sets how far the robot has driven per pulse the encoder reads. This value
	 * should be much lower than one, as there are usually hundreds of pulses per
	 * rotation.
	 * 
	 * To calculate, reset the encoders and push the robot forwards, say, five feet.
	 * Then count the number of pulses and do: (5x12)/pulses to get this in inches.
	 * 
	 * @param value
	 *            The encoder distance per pulse.
	 * @param encoder
	 *            Which encoder will be changed
	 */
	public void setEncoderDistancePerPulse(double value, MotorPosition encoder)
	{
		switch (encoder)
		{
		case ALL:
			for (int i = 0; i < encoders.length; i++)
				this.encoders[i].setDistancePerPulse(value);
			break;
		case LEFT_REAR:
			// set distance per pulse to the left rear encoder
			this.encoders[0].setDistancePerPulse(value);
			break;
		case RIGHT_REAR:
			// set distance per pulse to the right rear encoder
			this.encoders[1].setDistancePerPulse(value);
			break;
		case LEFT_FRONT:
			// set distance per pulse to the left front encoder
			this.encoders[2].setDistancePerPulse(value);
			break;
		case RIGHT_FRONT:
			// set distance per pulse to the right front encoder
			this.encoders[3].setDistancePerPulse(value);
			break;
		default:
			break;
		}
	}

	/**
	 * Sets the current gear of the robot, 0 being the lowest max being the highest.
	 * @param gear 
	 */
	public void setGear(int gear)
	{
		this.transmission.setGear(gear);
	}

	/**
	 * Sets a single gear to the given percentage.
	 * @param gear 0 is the lowest, increases.
	 * @param percent value from 0.0 to 1.0
	 */
	public void setGearPercentage(int gear, double percent)
	{
		this.transmission.setGearPercentage(gear, percent);
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

	/**
	 * Sets between what negative and positive percent the joystick 
	 * will return 0, and scales the remaining accordingly.
	 * @param value between 0.0 and 1.0
	 */
	public void setJoystickDeadband(double value)
	{
		this.transmission.setJoystickDeadband(value);
	}

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
	 * Shifts the robot's current software gears either down or up,
	 * based on a button. This also makes sure that if a button is
	 * held, it doesn't constantly cycle through gears.
	 * @param upShiftButton Button that controls the up shifting
	 * @param downShiftButton Button that controls the down shifting
	 */
	public void shiftGears(boolean upShiftButton, boolean downShiftButton)
	{
		this.transmission.shiftGears(upShiftButton, downShiftButton);
	}

	/**
	 * Set all motor percentages to 0.
	 */
	public void stop()
	{
		this.transmission.stop();
	}

	/**
	 * Strafe to a target using a mecanum transmission, and a gyro for
	 * stabilization. This will NOT be accurate because of mecanum's slippery
	 * properties.
	 * 
	 * @param inches
	 *            How far we should travel
	 * @param speed
	 *            How fast we should travel, in decimal percentage (0.0 to 1.0)
	 * @param directionDegrees
	 *            In which direction we should travel, 0 being forwards, -90 for
	 *            left and 90 for right.
	 * @return Whether or not we have finished strafing.
	 */
	public boolean strafeStraightInches(int inches, double speed, int directionDegrees)
	{
		// Wrong transmission type! we cant strafe if we dont have the right
		// transmission
		if (this.transmissionType != TransmissionType.OMNI_DIR)
			return true;

		// Reset the gyro and encoders on first start only
		if (strafeStraightInchesInit)
		{
			this.resetEncoders();
			this.gyro.reset();
			strafeStraightInchesInit = false;
		}

		// If we have traveled past the distance requested, then stop.
		if (this.getEncoderDistanceAverage(MotorPosition.ALL) > inches)
		{
			strafeStraightInchesInit = true;
			this.transmission.stop();
			return true;
		}
		// Run the rotation in a proportional loop based on the gyro.
		this.transmission.driveRaw(speed, Math.toRadians(directionDegrees), -(gyro.getAngle() * strafeStraightScalar));

		return false;
	}

	/**
	 * Turns the robot to a certain angle using the robot's turning circle to find
	 * the arc-length.
	 * 
	 * @deprecated Use instead {@link #turnDegrees(int, double, boolean)
	 *             turnDegrees(degrees, speed, usingGyro)}
	 * 
	 * @param angle
	 *            How far the robot should turn. Negative angle turns left, positive
	 *            turns right. (In Degrees)
	 * @param speed
	 *            How fast the robot should turn (0 to 1.0)
	 * @return Whether or not the robot has finished turning
	 */
	@Deprecated
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
		if (this.getEncoderDistanceAverage(MotorPosition.ALL) > Math.toRadians(Math.abs(angle)) * turningRadius)
		{
			// We have finished turning!
			this.transmission.stop();
			turnDegreesInit = true;
			return true;
		}

		// Change which way the robot turns based on whether the angle is
		// positive or negative
		if (angle < 0)
		{
			this.transmission.driveRaw(-speed, speed);
		} else
		{
			this.transmission.driveRaw(speed, -speed);
		}

		return false;
	}

	/**
	 * Turns the robot using the gyro, and slows down after passing
	 * "turnDegreesTriggerStage" degrees from the requested degrees.
	 * 
	 * @deprecated Slow and inaccurate at variable speeds. Use turnDegrees in
	 *             DrivePID instead for dynamic turning.
	 * 
	 * @param degrees
	 *            How much the robot should turn, in degrees. Positive for
	 *            clockwise, negative for counter-clockwise.
	 * @param power
	 *            How fast the robot should turn, in percentage (0.0 to 1.0)
	 * @return Whether or not the robot has finished turning.
	 */
	@Deprecated
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
			this.transmission.stop();
			turnDegrees2StageInit = true;
			return true;
		}

		// 2nd stage run slow
		if (Math.abs(this.gyro.getAngle()) > Math.abs(degrees) - turnDegreesTriggerStage)
		{
			this.transmission.driveRaw(Math.signum(degrees) * turnDegrees2ndStagePower,
					-Math.signum(degrees) * turnDegrees2ndStagePower);
		}
		// 1st stage run (power)
		else
		{
			this.transmission.driveRaw(Math.signum(degrees) * power, -Math.signum(degrees) * power);
		}

		return false;
	}

	/**
	 * Turns the robot based on values obtained from a gyroscopic sensor.
	 * 
	 * @deprecated Use instead {@link #turnDegrees(int, double, boolean)
	 *             turnDegrees(degrees, speed, usingGyro)}
	 * 
	 * @param angle
	 *            At what angle we should turn, in degrees. Negative is left,
	 *            positive is right.
	 * @param speed
	 *            How fast we should turn, in decimal percentage (0.0 to 1.0)
	 * @return Whether or not we have finished turning.
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
			this.transmission.stop();
			turnDegreesGyroInit = true;
			return true;
		}

		// Turn the robot based on whether we are going left or right.
		if (angle < 0)
		{
			this.transmission.driveRaw(-speed, speed);
		} else
		{
			this.transmission.driveRaw(speed, -speed);
		}

		return false;
	}

	/**
	 * Turns the robot on the spot a number of degrees requested.
	 * 
	 * @param degrees
	 *            How far the robot should turn. Positive for clockwise, negative
	 *            for counter-clockwise
	 * @param speed
	 *            How fast the robot should turn, from 0.0 to 1.0.
	 * @param acceleration
	 *            How much time, in seconds, the motors should accelerate to speed
	 *            over.
	 * @param usingGyro
	 *            If true, the gyro will be the chosen sensor. If false, the
	 *            encoders will be used.
	 * @return Whether or not the robot has finished turning the requested number of
	 *         degrees, used in a state machine.
	 */
	public boolean turnDegrees(int degrees, double speed, double acceleration, boolean usingGyro)
	{
		// Only on initialization, reset the sensor.
		if (turnDegreesInit == true)
		{
			if (usingGyro)
				this.gyro.reset();
			else
				this.resetEncoders();
			turnDegreesInit = false;
		}

		// If either sensor has reached the target position, then stop motors
		// and return true.
		if (usingGyro && Math.abs(this.gyro.getAngle()) > Math.abs(degrees) - gyroFudgeFactor)
		{
			this.transmission.stop();
			turnDegreesInit = true;
			return true;
		} else if (this.getEncoderDistanceAverage(MotorPosition.ALL) > degreesToEncoderInches(degrees, false))
		{
			this.transmission.stop();
			turnDegreesInit = true;
			return true;
		}

		// If degrees is positive, then turn left. If not, then turn right.
		if (degrees > 0)
			this.accelerateTo(speed, -speed, acceleration);
		else
			this.accelerateTo(-speed, speed, acceleration);

		return false;
	}

	// The transmission objects. Only one is used based on the transmission
	// object that is input.

	// The reason this is not one "TransmissionBase" object is that the drive
	// functions of each type require a different number of joysticks/input
	// values. Thus, inheritance is hard.

	/**
	 * Enum for deciding whether we're braking after driving, or turning.
	 * 
	 * @author Ryan McGee
	 *
	 */
	public static enum BrakeType
	{
		/** Braking after driving in a direction */
		AFTER_DRIVE,
		/** Braking after turning */
		AFTER_TURN
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

	private KilroyEncoder[] encoders;

	private GyroBase gyro = null;

	private final TransmissionBase transmission;

	private final TransmissionType transmissionType;

	// ================VARIABLES================

	// METHOD INITIALIZATION BOOLEANS

	private boolean arcInit = true;

	private boolean driveStraightInchesInit = true;

	private boolean driveInchesInit = true;

	private boolean pivotTurnDegreesInit = true;

	private boolean turnDegrees2StageInit = true;

	private boolean turnDegreesInit = true;

	private boolean strafeStraightInchesInit = true;

	private boolean turnDegreesGyroInit = true;

	// VARIABLES
	private int currentBrakeIteration = 0;

	private long driveStraightLastTime = 0;

	private long lastAccelerateTime = 0; // Milliseconds

	private long previousBrakeTime = 0; // milliseconds

	// TUNABLES
	private double accelMotorPower = 0;// Power sent to each motor

	private double accelStartingSpeed = .15;

	private int brakeDriveDeadband = 20; // ticks

	private double brakeDrivePower = .2;

	private int[] brakeMotorDirection = new int[4];

	private int[] brakePrevEncoderVals = new int[4];

	private int brakeTurnDeadband = 12;// ticks

	private double brakeTurnPower = 1.0;

	private double defaultAcceleration = .8;// Seconds

	// ---------------------------------
	// This is the distance we expect to move
	// during a brake call (in inches)
	// ---------------------------------
	private double distanceRequiredToBrake = 3.0;

	// How much should be subtracted from the left or right side while driving
	// straight
	private double driveStraightConstant = .1;

	private double gyroFudgeFactor = 3; // degrees

	private double pivotDegreesStationaryPercentage = .1;

	private double strafeStraightScalar = .08;

	private int totalBrakeIterations = 3;

	private double turnDegrees2ndStagePower = .19;

	private double turnDegreesTriggerStage = 40;// Degrees

	// The distance from the left side wheel to the right-side wheel divided by
	// 2, in inches. Used in turnDegrees.
	// Nov 4 changed from 16 to 17
	private static double turningRadius = 17;

	protected static final int INIT_TIMEOUT = 300;// Milliseconds until the
													// initialization should
													// reset.
}
