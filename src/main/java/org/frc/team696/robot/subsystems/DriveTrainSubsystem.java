/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  WPI_TalonSRX leftFront;
  WPI_TalonSRX leftMid;
  WPI_TalonSRX leftRear;

  WPI_TalonSRX rightFront;
  WPI_TalonSRX rightMid;
  WPI_TalonSRX rightRear;

  SpeedControllerGroup rightGroup;
  SpeedControllerGroup leftGroup;

  DifferentialDrive drive; 

  public DriveTrainSubsystem(int leftFrontPort, int leftMidPort, int leftRearPort, int rightFrontPort, int rightMidPort, int rightRearPort){
    leftFront = new WPI_TalonSRX(leftFrontPort);
    leftMid = new WPI_TalonSRX(leftMidPort);
    leftRear = new WPI_TalonSRX(leftRearPort);

    rightFront = new WPI_TalonSRX(rightFrontPort);
    rightMid = new WPI_TalonSRX(rightMidPort);
    rightRear = new WPI_TalonSRX(rightRearPort);

    rightGroup = new SpeedControllerGroup(rightFront, rightMid, rightRear);
    leftGroup = new SpeedControllerGroup(leftFront, leftMid, leftRear);

    drive = new DifferentialDrive(rightGroup, leftGroup);
  }

  public void execute(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
