/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  WPI_TalonSRX intakeA;
  WPI_TalonSRX intakeB;
  
  Solenoid sol;
  

  public IntakeSubsystem(int intakeAPort, int intakeBPort, int solPort){
   
    intakeA = new WPI_TalonSRX(intakeAPort);
    intakeB = new WPI_TalonSRX(intakeBPort);
    sol = new Solenoid(solPort);
  }

  public void runIntake(double speed){
    intakeA.set(speed);
    intakeB.set(speed);
}

  public void solIntake(boolean solenoid){
    sol.set(solenoid);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
