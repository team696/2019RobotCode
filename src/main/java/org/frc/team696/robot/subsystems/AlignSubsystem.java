/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import org.frc.team696.robot.commands.DriveCommand;
import org.frc.team696.robot.commands.DriveToAngleCommand;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class AlignSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static DriveCommand driveBack;
  public static DriveToAngleCommand driveToAngle;

  public void align(){
    driveBack =  new DriveCommand(-5000, 0);
    driveToAngle = new DriveToAngleCommand(45);
    int  caseNumber = 1;
    
    switch (caseNumber) {
      case 1:
        driveBack.start();
        caseNumber++;
        break;

      case 2:
        driveToAngle.start();
        //caseNumber++;
      default:
        break;
    }

    // if(driveBack.isCompleted()){
    // driveToAngle.start();
    // }

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
