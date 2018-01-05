
// ====================================================================
// FILE NAME: KilroyGyro.java (Team 339 - Kilroy)
//
// CREATED ON: sometime during 2017 build season
// CREATED BY: Becky BUtton
// MODIFIED ON:2/28/17
// MODIFIED BY: Ashley Espeland
// ABSTRACT:
// sets up and declares a gyro, includes various functions
// checks to see if we have a gyro and if we do, we declare it, if we
// dont then we print that to the screen, and if neither we return null
// calibration, reset, getAngle (in degrees), and getRate functions
// are included
//



package org.usfirst.frc.team339.HardwareInterfaces;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * sets up the gyro and includes the functions
 * 
 * @author Becky Button
 *
 */
public class KilroyGyro
{
private ADXRS450_Gyro gyro;

private boolean hasGyro = true;

/**
 * determines based on whether we have a gyro or not, whether to declare
 * the gyro(if we have one), print that we dont have a gyro(if we
 * dont), and return null if neither apply
 * 
 * @param hasGyro
 */
public KilroyGyro (boolean hasGyro)
{
    // set this.hasGyro equal to hasGyro-essentially setting up shorthand
    this.hasGyro = hasGyro;
    // if we do not have a gyro
    if (hasGyro == false)
        {
        // print out Gyro not connected
        System.out.println("Gyro not connected");
        }
    //// if we have a gyro
    if (hasGyro == true)
        {
        // then declare as a new gyro
        this.gyro = new ADXRS450_Gyro();
        // If the gyro is not slightly offset, we know it is not working.
        if (this.isConnected())
            {
            this.hasGyro = false;
            System.out.println("Gyro Not Connected");
            this.gyro = null;
            }
        }
    // if we don't have a gyro, but we don't not have a gyro, return null
    else
        // if neither then return null
        this.gyro = null;

    // IF for some reason the gyro is not created, THEN tell the class that we
    // do not have a gyro.
    if (this.gyro == null)
        {
        this.hasGyro = false;
        }
}

/**
 * calibration of the gyro
 */
public void calibrate ()
{
    // if we do not have a gyro
    if (this.isConnected() == false)
        {
        // then return void
        return;
        }
    // then calibrate gyro
    this.gyro.calibrate();
}

/**
 * resets the gyro
 */

public void reset ()
{
    // if we do not have a gyro
    if (this.isConnected() == false)
        {
        // return void
        return;
        }
    // then reset the gyro
    this.gyro.reset();
}

/**
 * will return the gyro angle in degrees
 * 
 * @return
 *         return gyro angle
 */

public double getAngle ()
{
    // if we don't have a gyro
    if (this.isConnected() == false)
        {
        // return 339339
        return 339339;
        }
    // return the angle of the gyro in degrees
    return this.gyro.getAngle();

}

/**
 * returns the rate of rotation for the gyro
 * 
 * @return
 *         return rate of rotation
 */

public double getRate ()
{
    // if we don't have a gyro
    if (this.isConnected() == false)
        {
        // return the random value 339339
        return 339339;
        }
    // return the rate of rotation of the gyro
    return this.gyro.getRate();
}

/**
 * 
 * @return Whether or not the gyro's angle is 0.0, which is returned
 *         if the SPI bus is null (in wpi's Gyro class)
 *         constructor.
 */
public boolean isConnected ()
{
    return this.hasGyro == true && this.gyro != null &&
            this.gyro.getAngle() != 0.0;

}

}
