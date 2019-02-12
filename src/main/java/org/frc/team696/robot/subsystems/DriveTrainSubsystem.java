/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  public WPI_VictorSPX leftFront;
  public WPI_VictorSPX leftMid;
  public WPI_VictorSPX leftRear;

  public WPI_VictorSPX rightFront;
  public WPI_VictorSPX rightMid;
  public WPI_VictorSPX rightRear;
  
  public SpeedControllerGroup leftSide;
  public SpeedControllerGroup rightSide;

  public DifferentialDrive drive;

  public DriveTrainSubsystem(int leftFrontPort, int leftMidPort, int leftRearPort, int rightRearPort, int rightMidPort, int rightFrontPort){
    leftFront = new WPI_VictorSPX(leftFrontPort);
    leftMid = new WPI_VictorSPX(leftMidPort);
    leftRear = new WPI_VictorSPX(leftRearPort);

    rightFront = new WPI_VictorSPX(rightFrontPort); 
    rightFront.setInverted(true);
    rightMid = new WPI_VictorSPX(rightMidPort);
    rightMid.setInverted(true);
    rightRear = new WPI_VictorSPX(rightRearPort);
    rightRear.setInverted(true);

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
