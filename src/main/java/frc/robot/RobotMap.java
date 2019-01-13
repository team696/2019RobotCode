/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {


  //drive ports
  public static int lFrontPort = 14;
  public static int lMiddlePort = 15;
  public static int lRearPort = 16;
  public static int rFrontPort = 1;
  public static int rMiddlePort = 2;
  public static int rRearPort = 3;


  public static int elevatorPortNumber = 12;
  public static int intakePortNumberA = 8;
  public static int intakePortNumberB = 9;
//IntakeSubsystems
 public static int elevatorBrakePortNumber = 3; 
 
 public static int tiltPortNumber = 0;
 public static int intakeSolPortNumber = 1;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
