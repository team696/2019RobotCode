/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import org.frc.team696.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveForward extends Command {

  private double initLeft, initRight, currentLeft, currentRight;
  private double encoder_ticks;

  public DriveForward(double encoder_ticks) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.encoder_ticks = encoder_ticks;
    requires(Robot.driveTrainSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initLeft = Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition();
    initRight = Robot.driveTrainSubsystem.rightFront.getSelectedSensorPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrainSubsystem.tankDrive(.5, .5);
    currentLeft = Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition();
    currentRight = Robot.driveTrainSubsystem.rightFront.getSelectedSensorPosition();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(currentLeft - initLeft >= encoder_ticks && currentRight - initRight >= encoder_ticks){
      Robot.driveTrainSubsystem.tankDrive(0, 0);
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