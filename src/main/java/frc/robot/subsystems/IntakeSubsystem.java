/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static TalonSRX intakeMotorA;
  static TalonSRX intakeMotorB;
 
   
  public IntakeSubsystem (int intakePortNumberA, int intakePortNumberB){
  
   intakeMotorA = new TalonSRX(intakePortNumberA);
   intakeMotorB = new TalonSRX(intakePortNumberB);
   }

   public void runIntake(double intakeSpeed) {
    intakeMotorA.set(ControlMode.PercentOutput, intakeSpeed);
    intakeMotorB.set(ControlMode.PercentOutput,intakeSpeed);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
