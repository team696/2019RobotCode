/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {

  public WPI_TalonSRX lFront;
  public WPI_TalonSRX lMiddle;
  public WPI_TalonSRX lRear;
  public WPI_TalonSRX rFront;
  public WPI_TalonSRX rMiddle;
  public WPI_TalonSRX rRear;

  DifferentialDrive diff;

  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide;

  public DriveSubsystem(int lFrontPort, int lMiddlePort, int lRearPort, int rFrontPort, int rMiddlePort, int rRearPort){
    
    lFront = new WPI_TalonSRX(lFrontPort);
    lMiddle = new WPI_TalonSRX(lMiddlePort);
    lRear =  new WPI_TalonSRX(lRearPort);
    rFront = new WPI_TalonSRX(rFrontPort);
    rMiddle = new WPI_TalonSRX(rMiddlePort);
    rRear = new WPI_TalonSRX(rRearPort);

    leftSide = new SpeedControllerGroup(lFront, lMiddle, lRear);
    rightSide = new SpeedControllerGroup(rFront, rMiddle, rRear);
    diff = new DifferentialDrive(leftSide, rightSide);
  }
    public void runDrive(double leftSpeed, double rightSpeed){
      diff.tankDrive(leftSpeed,rightSpeed);
    }
    

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
