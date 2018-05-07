package org.usfirst.frc.team339.HardwareInterfaces;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.Encoder;

public class KilroyEncoder
{
	private Encoder dioSensor = null;
	private BaseMotorController canSensor = null;
	private final SensorType type;

	public KilroyEncoder(int digitalPort1, int digitalPort2)
	{
		this.dioSensor = new Encoder(digitalPort1, digitalPort2);
		type = SensorType.D_IO;

	}

	public KilroyEncoder(BaseMotorController canMotorController)
	{
		this.canSensor = canMotorController;
		type = SensorType.CAN;
	}

	// public int getTicks()
	// {
	// switch (type)
	// {
	// case CAN:
	// return canSensor.getSelectedSensorPosition(0);
	// }
	// }

	public enum SensorType
	{
		CAN, D_IO
	}
}
