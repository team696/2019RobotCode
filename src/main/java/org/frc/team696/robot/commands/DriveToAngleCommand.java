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

public class DriveToAngleCommand extends Command {

  double tankLeft;
  double tankRight;

  double speed;
  double wheel;

  
  PIDController gyroController;
  PIDSource gyro;
  PIDOutput output;
  double gyrooutput; 

  double p = 0.015;
  double i = 0;
  double d = 0.07;


  public DriveToAngleCommand(double angle) {
    requires(Robot.driveTrainSubsystem);

    
    gyro = new PIDSource(){
    
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        
      }
    
      @Override
      public double pidGet() {
        return(Robot.navX.getYaw());
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
      }
    };

  output = new PIDOutput(){
  
    @Override
    public void pidWrite(double output) {
      gyrooutput=-output;
    }
  };

  gyroController = new PIDController(p, i, d, gyro, output);

  gyroController.setSetpoint(angle);
  gyroController.setPercentTolerance(5);



    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }




  @Override
  protected void initialize() {

    gyroController.reset();
    // Robot.navX.zeroYaw();
    gyroController.enable();
    


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    

    speed = 0;
    wheel = gyrooutput;

    tankLeft = speed - wheel;
    tankRight = speed + wheel;

    Robot.driveTrainSubsystem.tankDrive(tankLeft, tankRight);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return gyroController.onTarget();
    return gyroController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    gyroController.disable();
   
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //gyroController.disable();

  }
}
