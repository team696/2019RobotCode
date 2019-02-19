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
import org.frc.team696.robot.states.ClimberState;

public class ClimberAutoclimb extends Command {
  private boolean finished;

  public ClimberAutoclimb() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.finished = false;
    if(Robot.climber.getState() != ClimberState.ARMED){
      //Climber must be armed before climbing
      this.finished = true;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch(Robot.climber.getState()){
      case ARMED:
        Robot.climber.setState(ClimberState.CLIMBING);
        break;
      case CLIMBING:
        if(Robot.climber.getMaximumPositionError() < Climber.atHeightError){
          Robot.climber.setState(ClimberState.AT_HEIGHT);
        }
        break;
      case AT_HEIGHT:
        Robot.climber.setPusherPower(Climber.pusherPower);
        if(!Robot.climber.frontWOW()){
          //If no weight on front wheels, retract
          Robot.climber.setState(ClimberState.FRONT_ON_PLATFORM);
          Robot.climber.stowFront();
        }
        break;
      case FRONT_ON_PLATFORM:
        Robot.climber.stowFront();
        if(!Robot.climber.rearWOW()){
          Robot.climber.moveIndividual(0);
          Robot.climber.setState(ClimberState.STOWED);
        }
        break;
      default:
        this.finished = true;
        break;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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
