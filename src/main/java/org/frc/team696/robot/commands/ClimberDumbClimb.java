/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc.team696.robot.Robot;
import org.frc.team696.robot.subsystems.Climber;
import org.frc.team696.robot.subsystems.ClimberModule;

public class ClimberDumbClimb extends Command {
  public ClimberDumbClimb() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.climber.moveIndividual(ClimberModule.climbingPidSlot, Climber.frontDumbClimbPos, Climber.frontDumbClimbPos, Climber.rearDumbClimbPos, Climber.rearDumbClimbPos);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
