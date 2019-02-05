/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.frc.team696.robot.RobotMap;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class AlignPIDSubsystem extends PIDSubsystem {
  /**
   * Add your docs SubsystemNamehere.
   */

   WPI_TalonSRX leftFront;
   WPI_TalonSRX leftMid;
  public static WPI_TalonSRX leftRear;

   public static WPI_TalonSRX rightFront;
   WPI_TalonSRX rightMid;
   WPI_TalonSRX rightRear;

   SpeedControllerGroup leftSide, rightSide;

   DifferentialDrive diff;

  public AlignPIDSubsystem() {
    // Intert a subsystem name and PID values here
    super("AlignPIDSubsystem", 0.0009, 0, 0);
    setAbsoluteTolerance(10);
    leftFront = new WPI_TalonSRX(RobotMap.lFront);
    leftMid = new WPI_TalonSRX(RobotMap.lMid);
    leftRear = new WPI_TalonSRX(RobotMap.lRear);

    rightFront = new WPI_TalonSRX(RobotMap.rFront);
    rightMid = new WPI_TalonSRX(RobotMap.rMid);
    rightRear = new WPI_TalonSRX(RobotMap.rRear);

    leftSide = new SpeedControllerGroup(leftFront, leftMid, leftRear);
    rightSide = new SpeedControllerGroup(rightFront, rightMid, rightRear);

    diff = new DifferentialDrive(leftSide, rightSide);
    
    setSetpoint(500);
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
    return ((leftRear.getSelectedSensorPosition(0)+rightFront.getSelectedSensorPosition(0))/2);
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    
    diff.tankDrive(output,output);
  }
}
