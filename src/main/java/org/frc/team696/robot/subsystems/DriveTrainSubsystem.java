/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */

public class DriveTrainSubsystem extends Subsystem {
  
  public static CANSparkMax leftFront;
  public static CANSparkMax leftRear;

  public static CANSparkMax rightFront;
  public static CANSparkMax rightRear;
  
  public static SpeedControllerGroup leftSide;
  public static SpeedControllerGroup rightSide;

  public static DifferentialDrive drive;

  private static NetworkTableEntry ntflcurrent;
  private static NetworkTableEntry ntfrcurrent;
  private static NetworkTableEntry ntrlcurrent;
  private static NetworkTableEntry ntrrcurrent;

  public DriveTrainSubsystem(int leftFrontPort, int leftMidPort, int leftRearPort, int rightRearPort, int rightMidPort, int rightFrontPort){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable dtTable = inst.getTable("Drivetrain");
    
    leftFront = new CANSparkMax(leftFrontPort, MotorType.kBrushless);
    leftFront.restoreFactoryDefaults();
    leftFront.setIdleMode(IdleMode.kCoast);
    leftFront.setSmartCurrentLimit(40);
    
    leftRear = new CANSparkMax(leftRearPort, MotorType.kBrushless);
    leftRear.restoreFactoryDefaults();
    leftRear.setIdleMode(IdleMode.kCoast);
    leftRear.setSmartCurrentLimit(40);

    rightFront = new CANSparkMax(rightFrontPort, MotorType.kBrushless);
    rightFront.restoreFactoryDefaults();
    rightFront.setIdleMode(IdleMode.kCoast); 
    rightFront.setSmartCurrentLimit(40);
    rightFront.setInverted(true);

    rightRear = new CANSparkMax(rightRearPort, MotorType.kBrushless);
    rightRear.restoreFactoryDefaults();
    rightRear.setIdleMode(IdleMode.kCoast);
    rightRear.setSmartCurrentLimit(40);
    rightRear.setInverted(true);

    leftSide = new SpeedControllerGroup(leftFront, leftRear);
    rightSide = new SpeedControllerGroup(rightFront, rightRear);

    drive = new DifferentialDrive(leftSide, rightSide);
    drive.setDeadband(0.1);

    ntflcurrent = dtTable.getEntry("CurrentFL");
    ntfrcurrent = dtTable.getEntry("CurrentFR");
    ntrlcurrent = dtTable.getEntry("CurrentRL");
    ntrrcurrent = dtTable.getEntry("CurrentRR");


  }

  public void runDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
    ntflcurrent.setDouble(leftFront.getOutputCurrent());
    ntfrcurrent.setDouble(rightFront.getOutputCurrent());
    ntrlcurrent.setDouble(leftRear.getOutputCurrent());
    ntrrcurrent.setDouble(rightRear.getOutputCurrent());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
