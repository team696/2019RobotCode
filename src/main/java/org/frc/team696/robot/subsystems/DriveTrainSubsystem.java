/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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

  public DriveTrainSubsystem(int leftFrontPort, int leftMidPort, int leftRearPort, int rightRearPort, int rightMidPort, int rightFrontPort){
<<<<<<< HEAD
    
    leftFront = new WPI_VictorSPX(leftFrontPort);
    leftMid = new WPI_VictorSPX(leftMidPort);
    leftRear = new WPI_VictorSPX(leftRearPort);
=======
    leftFront = new CANSparkMax(leftFrontPort, MotorType.kBrushless);
    leftFront.restoreFactoryDefaults();
    leftFront.setIdleMode(IdleMode.kCoast);
    
    leftRear = new CANSparkMax(leftRearPort, MotorType.kBrushless);
    leftRear.restoreFactoryDefaults();
    leftRear.setIdleMode(IdleMode.kCoast);
>>>>>>> practicebot-master

    rightFront = new CANSparkMax(rightFrontPort, MotorType.kBrushless);
    rightFront.restoreFactoryDefaults();
    rightFront.setIdleMode(IdleMode.kCoast); 
    rightFront.setInverted(true);

    rightRear = new CANSparkMax(rightRearPort, MotorType.kBrushless);
    rightRear.restoreFactoryDefaults();
    rightRear.setIdleMode(IdleMode.kCoast);
    rightRear.setInverted(true);

    leftSide = new SpeedControllerGroup(leftFront, leftRear);
    rightSide = new SpeedControllerGroup(rightFront, rightRear);

    drive = new DifferentialDrive(leftSide, rightSide);
    drive.setDeadband(0.1);


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
