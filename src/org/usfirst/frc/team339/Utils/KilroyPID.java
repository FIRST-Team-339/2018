package org.usfirst.frc.team339.Utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A unifying PID class that allows the programmer to choose whether to process
 * PID loops on-board or off-board. The pros and cons for each are listed in
 * their respective constructor.
 * 
 * @author Ryan McGee
 *
 */
public class KilroyPID
{

	private final PIDController onBoardController;
	private final PIDSource sensor;
	private final BaseMotorController offBoardController;
	private final ControllerType type;

	/**
	 * Creates the KilroyPID object.
	 * 
	 * This constructor is for off-board CAN devices. It uses the CAN encoders /
	 * sensors attached directly to the motor controller. CAN motor controllers have
	 * PID loop calculations built in, leading to a smaller load on the RIO.
	 * 
	 * @param canMotorCont
	 *            The CAN motor controller w/ sensor attached.
	 */
	public KilroyPID(BaseMotorController canMotorCont)
	{

		offBoardController = canMotorCont;
		this.sensor = null;
		onBoardController = null;
		type = ControllerType.CAN;
	}

	/**
	 * Creates the KilroyPID object.
	 * 
	 * This constructor is for either PWM motors or CAN controllers using the
	 * "x_wpi" motor controller classes. It allows for a wider array of sensors, at
	 * the cost of PWM and sensor ports, and more strain on the RIO.
	 * 
	 * @param motorCont
	 *            The motor controller that will be manipulated by this class.
	 * @param sensor
	 *            The sensor that will pull data to compute the value sent to the
	 *            motor.
	 */
	public KilroyPID(SpeedController motorCont, PIDSource sensor)
	{
		onBoardController = new PIDController(0, 0, 0, sensor, motorCont);
		this.sensor = sensor;
		offBoardController = null;
		type = ControllerType.ONBOARD;
	}

	/**
	 * Sets the PIDF values for the current PID controller.
	 * 
	 * Sent motor value = (p * error) + (i * integral of error) + (d * derivative of
	 * error) + (f * setpoint)
	 * 
	 * While calculating the PID values for tuning, remember that the motor values
	 * is between -1023 and +1023, NOT -1 and 1.
	 * 
	 * @param p
	 *            The Proportional value: the error multiplied by this scalar
	 *            directly. This is good for speeding up correction based on how far
	 *            away the current position is from the setpoint.
	 * @param i
	 *            The Integral value: the cumulative error multiplied by this
	 *            scalar. Over time, if the current position is consistently off,
	 *            then this is good for making sure it eventually makes it to the
	 *            setpoint.
	 * @param d
	 *            The Derivative value: how fast the current position is nearing the
	 *            setpoint, times this scalar. This is good for making sure the
	 *            motor does not overshoot the setpoint by slowing it down as it
	 *            reaches it.
	 * @param f
	 *            The Feed-Forward value: A base line for what should be sent to the
	 *            motor. This is multiplied by the setpoint, and is used mostly by
	 *            veolocity loops.
	 */
	public void setPIDF(double p, double i, double d, double f)
	{
		this.p = p;
		this.i = i;
		this.d = d;
		this.f = f;
		switch (type)
		{
		case CAN:
			this.offBoardController.config_kP(0, p, 0);
			this.offBoardController.config_kI(0, i, 0);
			this.offBoardController.config_kD(0, d, 0);
			this.offBoardController.config_kF(0, f, 0);
			break;
		case ONBOARD:
			this.onBoardController.setPID(p, i, d, f);
			break;
		default:
			break;
		}
	}

	/**
	 * Sets the setpoint of the PID loop. Whatever value and type of value is input,
	 * the motor will use the PID loop and attempt to reach that value.
	 * 
	 * @param value
	 *            The value (in correct units based on the type) the motor will move
	 *            to.
	 * @param pidType
	 *            What the PID loop will see the value as. If it's position, then it
	 *            is a move and stop. If it's velocity, it will try to go the speed
	 *            input in value.
	 */
	public void setSetpoint(double value, PIDType pidType)
	{
		this.setpoint = value;
		this.lastSetpointType = pidType;
		// If the KilroyPID is not enabled, then don't set the setpoint in the controller itself.
		if (isEnabled == false)
			return;
		
		switch (type)
		{
		case CAN:
			// Set the type of movement and the setpoint.
			if (pidType == PIDType.POSITION)
				offBoardController.set(ControlMode.Position, value);
			else if (pidType == PIDType.VELOCITY)
				offBoardController.set(ControlMode.Velocity, value);
			break;
		case ONBOARD:
			// Set the type of movement
			if (pidType == PIDType.POSITION)
				this.sensor.setPIDSourceType(PIDSourceType.kDisplacement);
			else if (pidType == PIDType.VELOCITY)
				this.sensor.setPIDSourceType(PIDSourceType.kRate);
			// Set the setpoint
			this.onBoardController.setSetpoint(value);
			break;
		default:
			break;
		}
	}

	/**
	 * Either enables or disables the PID loop. If disabled, the motor will stop.
	 * 
	 * @param enabled
	 *            Whether it is enabled(true) or disabled(false)
	 */
	public void setEnabled(boolean enabled)
	{
		this.isEnabled = enabled;

		switch (type)
		{
		case CAN:
			if (enabled == false)
			{
				this.offBoardController.set(ControlMode.PercentOutput, 0);
			}
			else
			{
				this.setSetpoint(setpoint, lastSetpointType);
			}
			break;
		case ONBOARD:
			if (enabled == false)
			{
				this.onBoardController.disable();
			} else
			{
				this.onBoardController.enable();
				this.setSetpoint(setpoint, lastSetpointType);
			}
			break;
		default:
			break;
		}
	}

	/**
	 * If this PID controller is using a CAN sensor connected to the CAN motor
	 * controller, then this will invert it if it is "out of phase" (motor
	 * controller and sensor are not in the same direction)
	 */
	public void setCANSensorInverted(boolean inverted)
	{
		switch (type)
		{
		case CAN:
			this.offBoardController.setSensorPhase(inverted);
			break;
		default:
			break;

		}
	}

	/**
	 * If a PID is in use on a differential drive, for instance, then the front and
	 * rear motors can be tethered to one sensor / PID loop.
	 * 
	 * @param controller
	 *            Any CAN motor controller
	 */
	public void tetherCANMotorControllers(BaseMotorController... controller)
	{
		switch (type)
		{
		case CAN:
			for (BaseMotorController bmc : controller)
				bmc.follow(this.offBoardController);
			break;
		default:
			break;
		}
	}

	/**
	 * Clears the PID integral stored values, effectively "reseting" the PID loop.
	 */
	public void resetPID()
	{
		switch (type)
		{
		case CAN:
			this.offBoardController.setIntegralAccumulator(0, 0, 0);
			break;
		case ONBOARD:
			this.onBoardController.reset();
			break;
		default:
			break;
		}
	}

	public void resetContinuousSensor()
	{
		switch (type)
		{
		case CAN:
			this.offBoardController.setSelectedSensorPosition(0, 0, 0);
			break;
		case ONBOARD:
			if (this.sensor instanceof Encoder)
				((Encoder) sensor).reset();
			else if (this.sensor instanceof GyroBase)
				((GyroBase) sensor).reset();
			break;
		default:
			break;
		}
	}
	
	public void tunePIDSmartDashboard(String pidName)
	{
		if(initTune == false)
		{
			SmartDashboard.putNumber(pidName + "_p", 0);
			SmartDashboard.putNumber(pidName + "_i", 0);
			SmartDashboard.putNumber(pidName + "_d", 0);
			SmartDashboard.putNumber(pidName + "_f", 0);
			initTune = true;
		}
		p = SmartDashboard.getNumber(pidName + "_p", 0);
		i = SmartDashboard.getNumber(pidName + "_i", 0);
		d = SmartDashboard.getNumber(pidName + "_d", 0);
		f = SmartDashboard.getNumber(pidName + "_f", 0);
		
		this.setPIDF(p, i, d, f);
		
	}

	public enum ControllerType
	{
		CAN, ONBOARD
	}

	public enum PIDType
	{
		POSITION, VELOCITY, NULL
	}

	private double p, i, d, f, setpoint;
	private PIDType lastSetpointType = PIDType.NULL;
	private boolean isEnabled = true;
	private boolean initTune = false;
}
