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
public class HatchSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  // public static HatchSubsystem(){

  // }
  public static Solenoid moveHatchSol = new Solenoid(2);
  public static Solenoid actuateHatchSol = new Solenoid(0);

  public static void moveHatch(boolean bool){
    moveHatchSol.set(bool);
  }

  public static void actuateHatch(boolean bool){
    actuateHatchSol.set(bool);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
