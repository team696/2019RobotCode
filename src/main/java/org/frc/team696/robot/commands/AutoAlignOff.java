/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import org.frc.team696.robot.Robot;
import org.frc.team696.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class AutoAlignOff extends Command {
  private static boolean gotLeft = false;
  private static boolean gotRight = false;
  
  private static double y;
  private static double x = 779.423;
  
  private static double leftEncoder;
  private static double rightEncoder;

  private static double encoderDifference;
  private static double targetAngle;
  private static double targetAngleDegrees;

  private static boolean isFound = false;


  public AutoAlignOff() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("looking");
    if (Robot.leftIRSensor.get() && !gotLeft){
      gotLeft = true;
      leftEncoder = (Robot.driveTrainSubsystem.rightFront.getSelectedSensorPosition()+Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition()/2);  
      }

      if (!Robot.rightIRSensor.get() && !gotRight){
          gotRight = true;
          rightEncoder = (Robot.driveTrainSubsystem.rightFront.getSelectedSensorPosition()+Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition()/2);  
         // navX.zeroYaw();
          }
  
      if(gotLeft && gotRight){
          // finalEncoder = (driveTrainSubsystem.rightFront.getSelectedSensorPosition()+driveTrainSubsystem.leftRear.getSelectedSensorPosition())/2;
          // encoderDifference=finalEncoder-initialEncoder;
          encoderDifference = rightEncoder-leftEncoder;
          y = encoderDifference;
          targetAngle = Math.atan(x/y);

          targetAngleDegrees = Math.toDegrees(targetAngle);
          //navX.zeroYaw();
        isFound=true;
      }

      if(isFound){
       // new AutoAlignment(targetAngleDegrees).start();
      }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   return isFound;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
       //new AutoAlignment(targetAngleDegrees).start();
      //  new AutoAlignment(targetAngleDegrees).start();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
