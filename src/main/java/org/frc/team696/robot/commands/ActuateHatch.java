/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import org.frc.team696.robot.OI;
import org.frc.team696.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ActuateHatch extends Command {
  boolean bool;

  public ActuateHatch(boolean bool) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hatchSubsystem);
    this.bool = bool;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.hatchSubsystem.hatchActuator.get()==OI.operatorPanel.getRawButton(13)){
    Robot.hatchSubsystem.actuate(bool);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if(Robot.hatchSubsystem.hatchActuator.get()){
      // return true;
    // }
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
