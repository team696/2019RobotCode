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
  private static NetworkTableEntry ntflmotortemp;
  private static NetworkTableEntry ntfrmotortemp;
  private static NetworkTableEntry ntrlmotortemp;
  private static NetworkTableEntry ntrrmotortemp;

  public DriveTrainSubsystem(int leftFrontPort, int leftMidPort, int leftRearPort, int rightRearPort, int rightMidPort, int rightFrontPort){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable dtTable = inst.getTable("Drivetrain");
    
    leftFront = new CANSparkMax(leftFrontPort, MotorType.kBrushless);
    leftFront.restoreFactoryDefaults();
    leftFront.setIdleMode(IdleMode.kCoast);
    leftFront.setSmartCurrentLimit(40);
    leftFront.setOpenLoopRampRate(0);
    
    leftRear = new CANSparkMax(leftRearPort, MotorType.kBrushless);
    leftRear.restoreFactoryDefaults();
    leftRear.setIdleMode(IdleMode.kCoast);
    leftRear.setSmartCurrentLimit(40);
    leftRear.setOpenLoopRampRate(0);

    rightFront = new CANSparkMax(rightFrontPort, MotorType.kBrushless);
    rightFront.restoreFactoryDefaults();
    rightFront.setIdleMode(IdleMode.kCoast); 
    rightFront.setSmartCurrentLimit(40);
    rightFront.setInverted(true);
    rightFront.setOpenLoopRampRate(0);

    rightRear = new CANSparkMax(rightRearPort, MotorType.kBrushless);
    rightRear.restoreFactoryDefaults();
    rightRear.setIdleMode(IdleMode.kCoast);
    rightRear.setSmartCurrentLimit(40);
    rightRear.setInverted(true);
    rightRear.setOpenLoopRampRate(0);

    leftSide = new SpeedControllerGroup(leftFront, leftRear);
    rightSide = new SpeedControllerGroup(rightFront, rightRear);

    drive = new DifferentialDrive(leftSide, rightSide);
    drive.setDeadband(0.1);

    ntflcurrent = dtTable.getEntry("CurrentFL");
    ntfrcurrent = dtTable.getEntry("CurrentFR");
    ntrlcurrent = dtTable.getEntry("CurrentRL");
    ntrrcurrent = dtTable.getEntry("CurrentRR");

    ntflmotortemp = dtTable.getEntry("MotorTempFL");
    ntfrmotortemp = dtTable.getEntry("MotorTempFR");
    ntrlmotortemp = dtTable.getEntry("MotorTempRL");
    ntrrmotortemp = dtTable.getEntry("MotorTempRR");


  }

  public void runDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
    drive.feedWatchdog();
    ntflcurrent.setDouble(leftFront.getOutputCurrent());
    ntfrcurrent.setDouble(rightFront.getOutputCurrent());
    ntrlcurrent.setDouble(leftRear.getOutputCurrent());
    ntrrcurrent.setDouble(rightRear.getOutputCurrent());
    ntflmotortemp.setDouble(leftFront.getMotorTemperature());
    ntfrmotortemp.setDouble(rightFront.getMotorTemperature());
    ntrlmotortemp.setDouble(leftRear.getMotorTemperature());
    ntrrmotortemp.setDouble(rightRear.getMotorTemperature());
    //System.out.println(leftFront.get());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
