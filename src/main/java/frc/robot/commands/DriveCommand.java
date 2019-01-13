/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;


public class DriveCommand extends Command {
  public double targetDistance;
  public double targetDirection;

  public double errorDistance;
  public double errorDirection;

  public double currentDistance;
  public double currentDirection;

  public int loopNumber = 0;

  public double lFrontPos;
  public double lRearPos;
  public double rFrontPos;
  public double rRearPos;

  public double avgPos;

  public DriveCommand(/*double targetDistance, double targetDirection*/)  {

    //this.targetDistance = targetDistance;
    //this.targetDirection = targetDirection;
    requires(Robot.driveSubsystem);

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis)
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveSubsystem.lRear.setSelectedSensorPosition(0, 0, 20);
    Robot.driveSubsystem.rFront.setSelectedSensorPosition(0, 0, 20);

    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    Robot.driveSubsystem.runDrive(0.5,0.5);
    
    // errorDirection = targetDirection - currentDirection;
    // errorDistance = targetDistance - currentDistance;


     lRearPos = ((Robot.driveSubsystem.lRear.getSelectedSensorPosition(0))/200);
     rFrontPos = ((Robot.driveSubsystem.rFront.getSelectedSensorPosition(0))/200);
     
     avgPos=(lRearPos+rFrontPos)/2;
    System.out.println(avgPos);
    // System.out.println(currentDistance); 
    // // System.out.println(targetDistance);
    // // System.out.println(targetDirection);

    loopNumber++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if (Math.abs(errorDirection)<5 && Math.abs(errorDistance)<5){
    //   return true;
    // }
    // return false;
    if (avgPos>=55){  
      Robot.driveSubsystem.runDrive(0, 0);
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
