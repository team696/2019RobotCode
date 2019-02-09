/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.frc.team696.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveBack extends Command {

  private double initLeft, initRight, currentLeft, currentRight;
  private double encoder_ticks;

  public DriveBack(double encoder_ticks) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.encoder_ticks = encoder_ticks;
    requires(Robot.driveTrainSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Robot.driveTrainSubsystem.leftRear.setNeutralMode(NeutralMode.Coast);
    Robot.driveTrainSubsystem.rightFront.setNeutralMode(NeutralMode.Coast);

    initLeft = Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition();
    initRight = Robot.driveTrainSubsystem.rightFront.getSelectedSensorPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // Robot.driveTrainSubsystem.tankDrive(-.5, -.5);

    if(encoder_ticks > 0){
      Robot.driveTrainSubsystem.tankDrive(-.5, -.5);
    }else{
      Robot.driveTrainSubsystem.tankDrive(.5, .5);
    }
    currentLeft = Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition();
    currentRight = Robot.driveTrainSubsystem.rightFront.getSelectedSensorPosition();

    //System.out.println(currentLeft + " " + currentRight);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(currentLeft - initLeft) >= encoder_ticks && Math.abs(currentRight - initRight) >= encoder_ticks){
      Robot.driveTrainSubsystem.leftRear.setNeutralMode(NeutralMode.Brake);
      Robot.driveTrainSubsystem.rightFront.setNeutralMode(NeutralMode.Brake);
      Robot.driveTrainSubsystem.tankDrive(0, 0);
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Hello");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
