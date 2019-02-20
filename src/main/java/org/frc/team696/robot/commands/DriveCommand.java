/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import org.frc.team696.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCommand extends Command {


  double leftSpeed;
  double rightSpeed;


  double kPDistance;
  double kIDIstance;
  double kDDistance;

  double kPAngle;
  double kIAngle;
  double kDAngle;

  PIDOutput leftPidOutput;
  PIDOutput rightPidOutput;
  PIDOutput gyroPidOutput;

  double leftOutput;
  double rightOutput;
  double gyroOutput;

  PIDSource leftPidSource;
  PIDSource rightPidSource;
  PIDSource gyroPidSource;

  PIDController leftPidController;
  PIDController righPidController;
  PIDController gyroPidController;

  
  public DriveCommand(double targetDistance, double targetAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrainSubsystem);

    rightPidSource = new PIDSource(){
      PIDSourceType m_SourceType = PIDSourceType.kDisplacement;
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_SourceType=pidSource;
      }
    
      @Override
      public double pidGet() {
        return 0;
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return m_SourceType;
      }
    };

    rightPidOutput = new PIDOutput(){
    
      @Override
      public void pidWrite(double output) {
        rightOutput = output;
      }
    };



    leftPidSource = new PIDSource(){
      PIDSourceType m_SourceType = PIDSourceType.kDisplacement;

      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_SourceType = pidSource;
      }
    
      @Override
      public double pidGet() {
        return 0;
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return m_SourceType;
      }
    };

    leftPidOutput = new PIDOutput(){
    
      @Override
      public void pidWrite(double output) {
        leftOutput = output;
      } 
    };

    

    gyroPidSource = new PIDSource(){
    
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        
      }
    
      @Override
      public double pidGet() {
        return 0;
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return null;
      }
    };

    gyroPidOutput = new PIDOutput(){
    
      @Override
      public void pidWrite(double output) {
        
      }
    };
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

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
