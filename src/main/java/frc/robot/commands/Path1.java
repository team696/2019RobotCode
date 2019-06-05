/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Path1 extends Command {


String path1 = "path1";

Trajectory leftTrajectory;
Trajectory rightTrajectory;



  //  EncoderFollower leftFollower = new EncoderFollower(Robot.leftTrajectory1);
  //  EncoderFollower rightFollower = new EncoderFollower(Robot.rightTrajectory1);
  
  
  //   leftFollower.configureEncoder(leftIntPos, 10, wheelDiameter);
  //   leftFollower.configurePIDVA(0.5,0,0,1/16, 0);
  
  //   rightFollower.configureEncoder(rightIntPos , 10 , wheelDiameter);
  //   rightFollower.configurePIDVA(0.5, 0, 0, 1/16, 0);

  
  public Path1() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Notifier followerNotifier1 = new Notifier(this::followPath1);
    // Notifier followerNotifier1.startPeriodic(leftTrajectory1.get(0).dt);
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    
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
