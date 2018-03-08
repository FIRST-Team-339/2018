package org.usfirst.frc.team339.HardwareInterfaces.transmission;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * An extension of the Drive class that overrides the methods to use PID loops instead of static-correcting methods. 
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
		super(transmission, leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder, gyro);
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

	}

	private void init()
	{

	}

	// ======================Variables======================

	private double turnP = 0, turnI = 0, turnD = 0;

	private PIDSubsystem encoderTurnPID = new PIDSubsystem(turnP, turnI, turnD)
	{

		@Override
		protected double returnPIDInput()
		{
			return getEncoderDegreesTurned();
		}

		@Override
		protected void usePIDOutput(double output)
		{
			// TODO Auto-generated method stub

		}

		@Override
		protected void initDefaultCommand()
		{
			// TODO Auto-generated method stub

		}

	};

}
