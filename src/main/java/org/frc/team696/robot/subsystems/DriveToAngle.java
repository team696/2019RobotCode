/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import org.frc.team696.robot.Robot;
import org.frc.team696.robot.commands.DriveBackCommand;
import org.frc.team696.robot.commands.DriveCommand;
import org.frc.team696.robot.commands.DriveToAngleCommand;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class DriveToAngle extends Subsystem {

  public double targetAngle;
  public DriveToAngleCommand drivetoAngle;
  public DriveBackCommand driveBack;


  

  public DriveToAngle(double targetAngle){

    this.targetAngle = targetAngle;



  }


  public void goTargetAngle(){ 

    drivetoAngle = new DriveToAngleCommand(targetAngle);
    drivetoAngle.start();

  }

  public void driveBack(){
    driveBack = new DriveBackCommand(45);
    driveBack.start();
  }



  public void cancel(){
    // drivetoAngle.cancel();
    // driveToAngleSubsystem.cancel();
  }

  public void doAll(){
    // RnavX.zeroYaw();
    Robot.navX.zeroYaw();
    driveBack();
    if(driveBack.isCompleted()){
      driveBack.close();
      System.out.println("Hello");
      goTargetAngle(); 
    }
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
