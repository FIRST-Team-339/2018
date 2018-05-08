package org.usfirst.frc.team339.HardwareInterfaces;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * A sensor class that is able to use both CAN features and DIO features interchangeably. 
 * This allows for the hot-swapping of encoders physically, with minimal to no programming
 *  changes required.
 * @author Ryan McGee
 *
 */
public class KilroyEncoder implements PIDSource
{
	private Encoder dioSensor = null;
	private BaseMotorController canSensor = null;
	private final SensorType type;

	/**
	 * Create the KilroyEncoder object with a digital i/o encoder.
	 * Methods in this class are based on the WPI Encoder class.
	 * @param digitalPort1
	 * @param digitalPort2
	 */
	public KilroyEncoder(int digitalPort1, int digitalPort2)
	{
		this.dioSensor = new Encoder(digitalPort1, digitalPort2);
		type = SensorType.D_IO;

	}

	/**
	 * Creates the KilroyEncoder object with an encoder attached to a CAN motor controller.
	 * @param canMotorController The motor controller that the Encoder is attached to.
	 */
	public KilroyEncoder(BaseMotorController canMotorController)
	{
		this.canSensor = canMotorController;
		type = SensorType.CAN;
	}

	/**
	 * Encoders read revolutions / distances by counting a number of pulses based on how much 
	 * a shaft it is connecting to has rotated. Each pulse is known as a "tick".
	 * 
	 * @return the number of pulses that has taken place since last reset.
	 */
	public int get()
	{
		switch (type)
		{
		case CAN:
			return canSensor.getSelectedSensorPosition(0);
		case D_IO:
			return dioSensor.get();
		default:
			return 0;
		}
	}

	/**
	 * When we want to measure distance based on the encoder, we multiply by a scalar set earlier
	 * (usually during initialization), that translates rotation into linear movement.
	 * 
	 * @return how far the encoder has traveled based on a scalar.
	 */
	public double getDistance()
	{
		switch (type)
		{
		case CAN:
			return distancePerTick * this.get();
		case D_IO:
			return dioSensor.getDistance();
		default:
			return this.get();
		}
	}

	/**
	 * Rate is useful for determining the current speed of the the sensor. It is found
	 * by taking the derivative of the position, using past values to figure out the units 
	 * per second.
	 * 
	 * Useful also for applications such as continuous PID loops.
	 * 
	 * @return How fast the sensor is rotating / moving. It is given in [units] per second, 
	 * based on what you input for setDistancePerPulse.
	 */
	public double getRate()
	{
		switch (type)
		{
		case CAN:
			return (canSensor.getSelectedSensorVelocity(0) * 10) * distancePerTick;
		case D_IO:
			return dioSensor.getRate();
		default:
			return 0;
		}
	}

	/**
	 * @return What kind of sensor is being used, either CAN or D_IO.
	 */
	public SensorType getSensorType()
	{
		return this.type;
	}

	/**
	 * @return The CAN device, if being used, that contains the sensor.
	 *  If digital is being used, returns null.
	 */
	public BaseMotorController getAttachedCANDevice()
	{
		return canSensor;
	}

	/**
	 * @return the digital encoder object, as supplied by WPILib. 
	 * If CAN sensor is being used, returns null.
	 */
	public Encoder getAttachedDigitalDevice()
	{
		return dioSensor;
	}

	/**
	 * The Distance per pulse can be found one of two ways: 
	 * The first (and most precise option) is to calculate it, by using the 
	 * circumference formula: dpp = (PI * d) / p, where d is the diameter of the wheel, 
	 * and p is the pulses per revolution. Note that gearboxes will have to be considered as well.
	 * 
	 * The other option is to reset the encoder while the actuator is in a neutral position, and 
	 * move the actuater a set number of units. Read the number of ticks, and run the formula (x units)/(ticks).
	 * 
	 * @param value how far 1 tick is, translated to linear movement (usually inches).
	 */
	public void setDistancePerPulse(double value)
	{
		this.distancePerTick = value;
		switch (type)
		{
		case D_IO:
			dioSensor.setDistancePerPulse(value);
			break;
		default:
			return;
		}
	}

	/**
	 * Sets whether or not the sensor is reading backwards.
	 *  If so, it corrects by returning the reverse of whatever it is receiving.
	 * @param inverted Whether or not to invert reading of the encoder.
	 */
	public void setInverted(boolean inverted)
	{
		switch (type)
		{
		case CAN:
			canSensor.setSensorPhase(inverted);
			break;
		case D_IO:
			dioSensor.setReverseDirection(inverted);
			break;
		default:
			return;
		}
	}

	/**
	 * Sets any outstanding/stored values to 0.
	 */
	public void reset()
	{
		switch (type)
		{
		case CAN:
			canSensor.setSelectedSensorPosition(0, 0, 0);
			break;
		case D_IO:
			dioSensor.reset();
			break;
		default:
			return;
		}
	}

	/**
	 * Sets what the current PID source type is: displacement (position), or rate (velocity).
	 * This only determines what kind of value the PID loop will receive.
	 * @param pidSource Either kDisplacement or kRate
	 */
	@Override
	public void setPIDSourceType(PIDSourceType pidSource)
	{
		this.sourceType = pidSource;
	}

	/**
	 * @return Either kDisplacement or kRate: whatever was set by the user, or default kDisplacement.
	 */
	@Override
	public PIDSourceType getPIDSourceType()
	{
		return sourceType;
	}

	/**
	 * @return Either the velocity or position of the sensor, based on what was set by setPIDSourceType.
	 *  For use in the PIDController / KilroyPID object.
	 */
	@Override
	public double pidGet()
	{
		switch (sourceType)
		{
		case kDisplacement:
			return this.getDistance();
		case kRate:
			return this.getRate();
		default:
			return 0;
		}
	}

	/**
	 * What kind of sensor is attached.
	 * @author Ryan McGee
	 *
	 */
	public enum SensorType
	{
		/** Attached to a CAN Motor Controller*/
		CAN,
		/** Attached to the Digital I/O ports*/
		D_IO
	}

	private double distancePerTick = 1;
	private PIDSourceType sourceType = PIDSourceType.kDisplacement;

}
