
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends Subsystem {

  static TalonSRX elevatorMotor;

  public ElevatorSubsystem(int elevatorPortNumber){
    elevatorMotor = new TalonSRX(elevatorPortNumber);
    elevatorMotor.set(ControlMode.PercentOutput, 0);
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);   
  }
  public static void runElevatorUp(double elevatorSpeedUp){
    elevatorMotor.set(ControlMode.PercentOutput, elevatorSpeedUp);
  }
  public void runElevatorDown(double elevatorSpeedDown){
    elevatorMotor.set(ControlMode.PercentOutput,elevatorSpeedDown);
  }
  public void stopElevator(){
    elevatorMotor.set(ControlMode.PercentOutput,0);
  }

  public double getElevatorPosition(){
    return elevatorMotor.getSelectedSensorPosition(0);
  }

  public void zeroElevator(){
    elevatorMotor.setSelectedSensorPosition(0, 0, 20);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
