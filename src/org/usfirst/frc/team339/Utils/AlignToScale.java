package org.usfirst.frc.team339.Utils;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.HardwareInterfaces.KilroyGyro;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.Drive;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionBase;
import org.usfirst.frc.team339.vision.VisionProcessor;
import edu.wpi.first.wpilibj.Encoder;

public class AlignToScale extends Drive
{

private UltraSonic rearUltrasonic = null;




public AlignToScale (TransmissionBase transmission,
        Encoder leftFrontEncoder, Encoder rightFrontEncoder,
        Encoder leftRearEncoder, Encoder rightRearEncoder,
        UltraSonic ultrasonic, KilroyGyro gyro,
        VisionProcessor visionProcessor)
{
    super(transmission, leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder, ultrasonic, gyro,
            visionProcessor);

    this.rearUltrasonic = ultrasonic;

}

/**
 * Hopefully will align to proper distance w. scale
 * then raise fork lift and eject cube
 * 
 * 
 * param speed
 * Set to negative for too close ajustment
 * 
 * @return true when completed
 * 
 * 
 */

public boolean alignToScale (double speed, double deadband)
{
    // Started align to scale
    // todo optimize deadband to distance

    // checks if in proper distance
    // ROBOT_TO_SCALE_DISTANCE 68-36 =32
    if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE

            && this.rearUltrasonic
                    .getDistanceFromNearestBumper() >= ROBOT_TO_SCALE_DISTANCE
                            - deadband)
        {
        System.out.println("Our distance to the scale is correct");

        // Hardware.tractionDrive.drive(0, 0);
        speed = 0;
        aligned = true;
        // start the move forklift switch

        if (Hardware.cubeManipulator.scoreSwitch())
            {
            return true;
            }



        }
    // if to far from scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() < ROBOT_TO_SCALE_DISTANCE
                    - deadband
    /* && aligned == false */)
        {
        System.out.println("We are too close to the scale");
        Hardware.cubeManipulator.setLiftPosition(0, 0);
        Hardware.transmission.drive(speed, speed);
        }
    // if to close to scale
    else if (this.rearUltrasonic
            .getDistanceFromNearestBumper() > ROBOT_TO_SCALE_DISTANCE
    /* && aligned == false */)
        {
        System.out.println("We are to far from the scale");
        Hardware.cubeManipulator.setLiftPosition(0, 0);
        Hardware.transmission.drive(-speed, -speed);
        }
    return false;
}

boolean aligned = false;

private static final double SCALE_TO_WALL_DISTANCE = 68;

private static final double ROBOT_LENGTH = 36;// TODO magic number

private static final double ROBOT_TO_SCALE_DISTANCE = SCALE_TO_WALL_DISTANCE
        - ROBOT_LENGTH;

}
