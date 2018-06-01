package org.usfirst.frc.team339.Utils.drive;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase.MotorPosition;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * This is just a useless class to do some testing. DO NOT USE IN COMPETITION
 * 
 * ...that would be bad...
 * 
 * @author Ryan McGee
 *
 */
public class DriveTesting
{
private static boolean bangBangInit = true;

public static boolean bangBangDriveInches (double speed,
        double distance, double deadband)
{
    if (bangBangInit)
        {
        Hardware.drive.resetEncoders();
        bangBangInit = false;
        }

    double encoders = Hardware.drive
            .getEncoderDistanceAverage(MotorPosition.ALL);

    if (encoders > distance)
        {
        Hardware.transmission.driveRaw(-speed, -speed);
        }
    else
        {
        Hardware.transmission.driveRaw(speed, speed);
        }

    if (Math.abs(encoders - distance) < deadband)
        {
        Hardware.transmission.stop();
        bangBangInit = true;
        return true;
        }

    return false;
}

public static PIDSubsystem testingPID = new PIDSubsystem(0, 0, 0)
{
@Override
protected double returnPIDInput ()
{
    return (Hardware.leftFrontDriveEncoder.getDistance()
            + Hardware.rightFrontDriveEncoder.getDistance()) / 2.0;
}

@Override
protected void usePIDOutput (double output)
{
    pidOutput = output;
}

@Override
protected void initDefaultCommand ()
{
}
};

private static boolean pidDriveInchesInit = true;

private static double pidOutput = 0;

public static boolean pidDriveInches (double speed, double distance,
        double acceleration, double tolerance)
{
    if (pidDriveInchesInit)
        {
        Hardware.drive.resetEncoders();
        testingPID.setAbsoluteTolerance(tolerance);
        // testingPID.getPIDController().setInputRange(-speed, speed);
        testingPID.getPIDController().reset();
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

    Hardware.drive.accelerateTo(pidOutput, pidOutput, acceleration);

    return false;
}

public static void reset ()
{
    testingPID.setName("Testing PID Controller");
    testingPID.disable();
    testingPID.getPIDController().reset();
    pidDriveInchesInit = true;
    bangBangInit = true;
    Hardware.drive.reset();
}

}
