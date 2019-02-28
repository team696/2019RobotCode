/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class HatchIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Solenoid hatchTilt;
  Solenoid hatchIntake;

  public static enum HatchTiltPos{
    high, low
  }

  public static enum HatchIntakePos{
    open, closed
  }

  public HatchIntakeSubsystem(int hatchTiltPort, int hatchIntakePort){
    hatchTilt = new Solenoid(hatchTiltPort);
    hatchIntake = new Solenoid(hatchIntakePort);
  }

  public void runHatchTilt(HatchTiltPos hatchTiltPos){
    
    switch(hatchTiltPos){
        case high:
        hatchTilt.set(true);
        break;

        case low:
        hatchTilt.set(false);
        break;

    }

  }

  public void runHatchIntake(HatchIntakePos hatchIntakePos){

    switch(hatchIntakePos){
      case open:
      hatchIntake.set(true);
      break;
      
      case closed:
      hatchIntake.set(false);
      break;
    }
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
