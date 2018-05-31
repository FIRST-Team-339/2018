package org.usfirst.frc.team339.Utils.drive;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * This is just a useless class to do some testing. DO NOT USE IN COMPETITION
 * 
 * ...that would be bad...
 * @author Ryan McGee
 *
 */
public class DriveTesting
{
	private static boolean bangBangInit = true;

	public static boolean bangBangDriveInches(double speed, double distance, double deadband)
	{
		if (bangBangInit)
		{
			Hardware.drive.resetEncoders();
			bangBangInit = false;
		}

		double encoders = Hardware.drive.getEncoderDistanceAverage(MotorPosition.ALL);

		if (encoders > Math.abs(distance - deadband) && encoders < Math.abs(distance + deadband))
		{
			Hardware.transmission.stop();
			bangBangInit = true;
			return true;
		}

		Hardware.transmission.driveRaw(speed, speed);

		return false;
	}

	public static PIDSubsystem testingPID = new PIDSubsystem(0, 0, 0)
	{
		@Override
		protected double returnPIDInput()
		{
			return (Hardware.leftFrontDriveEncoder.getDistance() + Hardware.rightFrontDriveEncoder.getDistance()) / 2.0;
		}

		@Override
		protected void usePIDOutput(double output)
		{
			Hardware.transmission.driveRaw(output, output);
		}

		@Override
		protected void initDefaultCommand()
		{
		}
	};

	private static boolean pidDriveInchesInit = true;

	public static boolean pidDriveInches(double p, double i, double d, double tolerance, double distance,
			double acceleration)
	{
		if (pidDriveInchesInit)
		{
			testingPID.setName("Testing PID Controller");
			Hardware.drive.resetEncoders();
			testingPID.getPIDController().reset();
			testingPID.getPIDController().setPID(p, i, d);
			testingPID.setAbsoluteTolerance(tolerance);
			testingPID.setSetpoint(distance);
			testingPID.enable();
			pidDriveInchesInit = false;
		}

		if (testingPID.onTarget())
		{
			testingPID.disable();
			pidDriveInchesInit = true;
			return true;
		}
		return false;
	}

	public static void reset()
	{
		testingPID.getPIDController().reset();
		pidDriveInchesInit = true;
		bangBangInit = true;
	}

}
