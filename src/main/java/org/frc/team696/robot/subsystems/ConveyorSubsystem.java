/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ConveyorSubsystem extends Subsystem {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public  WPI_VictorSPX topConveyorMotor;
  public  WPI_VictorSPX bottomConveyorMotor;
  public Solenoid conveyorTilt;
  
public ConveyorSubsystem(int topConveyorMotorPort, int botomConveyorMotorPort, int conveyorSolPort){
  topConveyorMotor = new WPI_VictorSPX(topConveyorMotorPort);
  bottomConveyorMotor = new WPI_VictorSPX(botomConveyorMotorPort);
  conveyorTilt = new Solenoid(conveyorSolPort);
}

public void runConveyor(double conveyorSpeed){
  topConveyorMotor.set(conveyorSpeed);
  bottomConveyorMotor.set(conveyorSpeed);

}
public void tiltConveyor(boolean conveyorState){
  conveyorTilt.set(conveyorState);
} 


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
