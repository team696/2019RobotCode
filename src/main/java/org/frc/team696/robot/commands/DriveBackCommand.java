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

public class DriveBackCommand extends Command {

  int loopNum;
  int step = 0;
  double angle;

  DriveToAngleCommand command;

  public DriveBackCommand(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrainSubsystem);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    command = new DriveToAngleCommand(-angle);
    loopNum = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println(loopNum);
    switch(step){

      case 0:
        loopNum++;
        Robot.driveTrainSubsystem.tankDrive(-0.5, -0.5);
        if(loopNum >= 50){
          Robot.driveTrainSubsystem.tankDrive(0, 0);
          loopNum = 0;
          step++;
          System.out.println(step);
        }
        break;

      case 4:
        command.start();
        command.cancel();
        step++;
        break;

      case 1:
        loopNum++;
        Robot.driveTrainSubsystem.tankDrive(0.5, 0.5);
        if(loopNum >= 25){
          Robot.driveTrainSubsystem.tankDrive(0, 0);
          loopNum = 0;
          step++;
        }
        break;

      case 3:
        




    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(step == 3){
      System.out.println("done");
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
