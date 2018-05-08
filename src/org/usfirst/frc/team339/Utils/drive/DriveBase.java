package org.usfirst.frc.team339.Utils.drive;

import org.usfirst.frc.team339.HardwareInterfaces.KilroyEncoder;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase;

/**
 * The Base class that controls standard functions needed to autonomously drive
 * the robot based on sensors.
 * 
 * @author Ryan McGee
 *
 */
public class DriveBase
{
	private KilroyEncoder[] encoders;
	private TransmissionBase transmisson;

	public DriveBase(TransmissionBase transmission, KilroyEncoder leftEncoder, KilroyEncoder rightEncoder)
	{	
		this.encoders = new KilroyEncoder[2];
		this.encoders[0] = leftEncoder;
		this.encoders[1] = rightEncoder;
		this.transmisson = transmission;
	}

	public DriveBase(TransmissionBase transmission, KilroyEncoder leftRearEncoder, KilroyEncoder rightRearEncoder,
			KilroyEncoder leftFrontEncoder, KilroyEncoder rightFrontEncoder)
	{
		this.encoders = new KilroyEncoder[4];
		this.encoders[0] = leftRearEncoder;
		this.encoders[1] = rightRearEncoder;
		this.encoders[2] = leftFrontEncoder;
		this.encoders[3] = rightFrontEncoder;
		this.transmisson = transmission;
	}

}
