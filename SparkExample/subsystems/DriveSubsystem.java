/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.DriveCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  
  double GEAR_RATIO = 4.821;//6.57992
  double circumfrence = 18.84;
  double INCHES_PER_REV = circumfrence/GEAR_RATIO;

  int timeoutMs = 10;

  // Front is the follow motor, and it is based on following the primary motor of its side.
  public CANSparkMax leftDrivePrimary = new CANSparkMax(1, MotorType.kBrushless),
                     leftDriveFront = new CANSparkMax(2, MotorType.kBrushless),
                     rightDriveFront = new CANSparkMax(9, MotorType.kBrushless),
                     rightDrivePrimary = new CANSparkMax(10, MotorType.kBrushless);

  
  public CANPIDController leftPID = new CANPIDController(leftDrivePrimary);
  public CANPIDController rightPID = new CANPIDController(rightDrivePrimary);

  public CANEncoder leftEncoder = new CANEncoder(leftDrivePrimary);
  public CANEncoder rightEncoder = new CANEncoder(rightDrivePrimary);


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveCommand());

    leftDriveFront.follow(leftDrivePrimary);
    rightDriveFront.follow(rightDrivePrimary);
  
    leftDrivePrimary.setCANTimeout(timeoutMs);
    rightDrivePrimary.setCANTimeout(timeoutMs);
    leftDrivePrimary.setMotorType(MotorType.kBrushless);
    rightDrivePrimary.setMotorType(MotorType.kBrushless);

  }

  public void set(double left, double right) {
    leftDrivePrimary.set(left);
    rightDrivePrimary.set(right);
  }

  public double getLeftRevs() {
    return leftEncoder.getPosition();
  }

  public double getRightRevs() {
    return rightEncoder.getPosition();
  }

  public double inchesToRevs(double INCHES) {
    return INCHES/INCHES_PER_REV;
  }

  // Takes an input of ticks
  public void setPID(double left, double right) {
    leftPID.setReference(left, ControlType.kPosition);
    rightPID.setReference(right, ControlType.kPosition);
  }

  public void stop() {
    leftDrivePrimary.set(0);
    rightDrivePrimary.set(0);
  }
}
