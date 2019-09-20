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
public class HatchSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Solenoid hatchActuator;
  public Solenoid hatchPosition;

  public HatchSubsystem(final int hatchActuator, final int hatchPosition){
    this.hatchActuator = new Solenoid(17, hatchActuator);
    this.hatchPosition = new Solenoid(17, hatchPosition);
  }

  public void actuate(boolean bool){
      hatchActuator.set(bool);
  }

  int loopNumber = 0;
  public void move(boolean bool){
    // if(!bool){
    //   hatchActuator.set(true);
    //   loopNumber = 0;
    // }else{
    //   loopNumber++;
    //   if(loopNumber == 16){
    //     hatchActuator.set(false);
    //   }
    // }

    hatchPosition.set(bool);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
