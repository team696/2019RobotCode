/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  WPI_VictorSPX leftFront;
  WPI_VictorSPX leftMid;
  WPI_VictorSPX leftRear;

  WPI_VictorSPX rightFront;
  WPI_VictorSPX rightMid;
  WPI_VictorSPX rightRear;
  
  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide;

  DifferentialDrive drive;

  public DriveTrainSubsystem(int leftFrontPort, int leftMidPort, int leftRearPort, int rightFrontPort, int rightMidPort, int rightRearPort){
    leftFront = new WPI_VictorSPX(leftFrontPort);
    leftMid = new WPI_VictorSPX(leftMidPort);
    leftRear = new WPI_VictorSPX(leftRearPort);

    rightFront = new WPI_VictorSPX(rightFrontPort);
    rightMid = new WPI_VictorSPX(rightMidPort);
    rightRear = new WPI_VictorSPX(rightRearPort);

    leftSide = new SpeedControllerGroup(leftFront, leftMid, leftRear);
    rightSide = new SpeedControllerGroup(rightFront, rightMid, rightRear);

    drive = new DifferentialDrive(leftSide, rightSide);


  }

  public void runDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
