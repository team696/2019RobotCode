/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class DriveCommand extends Command {

  int timeoutMs = 10;

  double left;
  double right;

  public DriveCommand() {
    requires(Robot.DRIVE_SUBSYSTEM);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.DRIVE_SUBSYSTEM.leftDrivePrimary.setCANTimeout(timeoutMs);
    Robot.DRIVE_SUBSYSTEM.rightDrivePrimary.setCANTimeout(timeoutMs);
    Robot.DRIVE_SUBSYSTEM.leftDrivePrimary.setMotorType(MotorType.kBrushless);
    Robot.DRIVE_SUBSYSTEM.rightDrivePrimary.setMotorType(MotorType.kBrushless);
    System.out.println("Initialized DRIVE_SUBSYSTEM.");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    left = OI.driverController.getRawAxis(1);
    right = -OI.driverController.getRawAxis(5);


    if(Math.abs(left) < .1) {
      left = 0;
    } 
    
    if(Math.abs(right) < .1) {
      right = 0;
    }

    Robot.DRIVE_SUBSYSTEM.set(left, right);

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
