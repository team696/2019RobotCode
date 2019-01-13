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

public class DriveTrainSubsystem extends Subsystem {

  public WPI_TalonSRX leftRear;
  public WPI_TalonSRX leftMid;
  public WPI_TalonSRX leftFront;

  public WPI_TalonSRX rightRear;
  public WPI_TalonSRX rightMid;
  public WPI_TalonSRX rightFront;

  public SpeedControllerGroup leftSide;
  public SpeedControllerGroup rightSide;

  private DifferentialDrive drive;


  public DriveTrainSubsystem(int leftRear, int leftMid, int leftFront,
                             int rightRear, int rightMid, int rightFront) {

    /*
        Initializing TalonSRX Objects and PID Controller
    */

    this.leftRear = new WPI_TalonSRX(leftRear);
    this.leftMid = new WPI_TalonSRX(leftMid);
    this.leftFront = new WPI_TalonSRX(leftFront);

    this.rightRear = new WPI_TalonSRX(rightRear);
    this.rightMid = new WPI_TalonSRX(rightMid);
    this.rightFront = new WPI_TalonSRX(rightFront);

    /*
      Assigning each individual TalonSRX to their corresponding groups.
    */

    leftSide = new SpeedControllerGroup(this.leftRear, this.leftMid, this.leftFront);
    rightSide = new SpeedControllerGroup(this.rightRear, this.rightMid, this.rightFront);
          
    /*
      Initializing the differential drive object with leftSide and rightSide
    */
    
    this.drive = new DifferentialDrive(this.leftSide, this.rightSide);

  }

  /**
   * Drive function for the robot
   * @param leftDrive value to apply to the left drive motors of the robot
   * @param rightDrive value to apply to the right drive motors of the robot
   */

  public void tankDrive(double leftDrive, double rightDrive) {
    drive.tankDrive(leftDrive, rightDrive);
  }

  @Override
  public void initDefaultCommand() {

  }
}
