/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.frc.team696.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * Add your docs here.
 */
public class LeftPIDSubsystem extends PIDSubsystem {
  /**
   * Add your docs here.
   */


  WPI_TalonSRX leftRear =  new WPI_TalonSRX(RobotMap.lRear);
  WPI_TalonSRX leftMid = new WPI_TalonSRX(RobotMap.lMid);
  WPI_TalonSRX leftFront = new WPI_TalonSRX(RobotMap.lFront);

  public LeftPIDSubsystem() {
    
    // Intert a subsystem name and PID values here
    super("LeftPIDSubsystem", 0.00009, 0, 0);
    setAbsoluteTolerance(10);
    getPIDController().setContinuous(false);
    setSetpoint(5000);
    // setSetpoint(5000);
    // enable();
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
     
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return leftRear.getSelectedSensorPosition(0);
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    leftRear.pidWrite(output);
    leftMid.pidWrite(output);
    leftFront.pidWrite(output);
  }
}
