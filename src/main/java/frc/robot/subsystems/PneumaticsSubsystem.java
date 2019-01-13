/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class PneumaticsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Solenoid discBrake;
  Solenoid tiltSol;
  static Solenoid intakeBrake;
  
  public  PneumaticsSubsystem(int brakePortNumber, int tiltPortNumber, int intakeBrakePortNumber){
    discBrake = new Solenoid(brakePortNumber);
    tiltSol = new Solenoid(tiltPortNumber);
    intakeBrake = new Solenoid(intakeBrakePortNumber);
    
  }
  public void runDiscBrake(boolean isBrake){
    discBrake.set(isBrake);
  }
  public void runIntakeBrake(boolean isIntakeBrake){
    intakeBrake.set(isIntakeBrake);
  }
  public void runTilt(boolean isTilt){
    tiltSol.set(isTilt);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
