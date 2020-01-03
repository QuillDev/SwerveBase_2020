/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import SKCommon.hardware.*;
import SKCommon.swerve.*;
import SKCommon.swerve.SwerveDrivetrain.Drivemode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

   //create the joystick so we can use it in all of our control sections
   private Joystick driveStick = new Joystick(0); //joystick for driving the robot

   //gyro on the robot for field oriented control
   private qGyro gyro = new qGyro();

   //Create the swerve drivetrain using the four modules
   private SwerveDrivetrain swerve = new SwerveDrivetrain();

   //Accelerometer built into RoboRIO
   private RioAccel accelerometer = new RioAccel();

   //create field oriented booelan
   private boolean fieldOriented = true;

  @Override
  public void robotInit() {
    //set the heading of the gyro to zero when the robot is turned on
    gyro.reset();
    gyro.calibrate();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    //get the gryo angle and store it as fieldOffset
    double fieldOffset = gyro.readAngleInRadians();

    //get the 3 degrees of freedom from the joystick that we can use to drive
    double fwd = -driveStick.getRawAxis(1); // We get the negative because the stick is inversed
    double str = driveStick.getRawAxis(0); // Get the x value of the left stick to use as the strafe
    double rcw = driveStick.getRawAxis(4); // get the x2 value for rotating the robot

    //change fieldmode depending on whether button has been pressed
    if(driveStick.getRawButtonPressed(1)){
      //flips the value of fieldOriented a is pressed
      fieldOriented = !fieldOriented;
    }

    //drive the swerve drive (using simple control mode)
    swerve.drive(fwd, str, rcw, fieldOffset, fieldOriented, SwerveDrivetrain.Drivemode.kDriveSimple);

    //print all dashboard information
    setupDashboard();

  }

  @Override
  public void testInit() {
  }

  @Override
  //We'll use this as the testing period for swerve modules
  public void testPeriodic() {

    //get the gryo angle and store it as fieldOffset
    double fieldOffset = gyro.readAngleInRadians();

    //don't move any motors and set their angle to 0 [Should be Straight]
    swerve.drive(0, 0, 0, fieldOffset, true, Drivemode.kDriveSimple);

    //print data to the dashboard
    setupDashboard();
  }

  /**
   * Print all of the different things we want information on to the dashboard
   */
  public void setupDashboard() {
    //print accelerometer data
    accelerometer.printDashboard();

    //print gyro data to dashboard
    gyro.printDashboard();
  }

}
