/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;
import org.frc.team696.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;


public class CorrectDriveLeft extends Command {

  double adjustment;




  
  public CorrectDriveLeft(double adjustment) {
    this.adjustment = adjustment;
    requires(Robot.drive);

  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {

    Robot.drive.tankDrive(Robot.leftSide + adjustment, Robot.rightSide + adjustment);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
