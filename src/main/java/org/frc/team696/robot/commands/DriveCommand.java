/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import org.frc.team696.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/* ----------------------------------------------------------------------------------

    PID DriveCommand by Justin G.
    ~Uses the WPIlib PIDController~

   ----------------------------------------------------------------------------------*/
public class DriveCommand extends Command {

  /*Left and Right parameters of tank drive method*/
  double tankLeft;
  double tankRight;
 
  /*P, I, and D values for Distance(How far the robot will drive)*/
  double pDistance = 0.00009;
  double iDistance = 0.00;
  double dDistance = 0.000;

  /*P, I, and D values for Direction(Angle the robot will turn)*/
  double pDirection = 0.015; //0.015
  double iDirection = 0.0;
  double dDirection = 0.07; //0.07


  /* Doubles that are set to the output that the pidController gives.
    We set these outputs to the 'speed' and 'wheel' variables  */
  double leftPidOutput;
  double rightPidOutput;
  double gyroPidOutput;


  /* Allows for the PID to output a value. Is parameter for PIDController object*/
  PIDOutput leftSide;
  PIDOutput rightSide;
  PIDOutput pidOutputGyro;
  

  /* Allows you to set the source that the PID will use. For example, encoders or gyros */
  PIDSource pidSourceRight;
  PIDSource pidSourceLeft;
  PIDSource pidSourceGyro;

  /*The Object for the PID Controller.*/
  PIDController PIDLeftDistance;
  PIDController PIDRightDistance;
  PIDController PIDGyro;





  




  // double speedFactor = 0.6;


  public DriveCommand(double distance, double direction) {
    requires(Robot.driveTrainSubsystem);


    pidSourceRight = new PIDSource(){
      PIDSourceType m_SourceType = PIDSourceType.kDisplacement;
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_SourceType = pidSource;
      }
      @Override
      public double pidGet() {
        return (-Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition(0)) ;
      }
      @Override
      public PIDSourceType getPIDSourceType() {
        return m_SourceType;
      }
    };


    rightSide  = new PIDOutput(){
      @Override
      public void pidWrite(double output) {
        rightPidOutput = output;
      }
    };


    pidSourceLeft = new PIDSource(){
      PIDSourceType m_SourceType = PIDSourceType.kDisplacement;
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_SourceType = pidSource;
      }
      @Override
      public double pidGet() {
        return (Robot.driveTrainSubsystem.rightFront.getSelectedSensorPosition(0));
      }
      @Override
      public PIDSourceType getPIDSourceType() {
        return m_SourceType;
      }
    };



    leftSide = new PIDOutput(){
      @Override
      public void pidWrite(double output) {
        leftPidOutput = output;
      }
    };


    pidSourceGyro = new PIDSource(){
      PIDSourceType m_SourceType = PIDSourceType.kDisplacement;
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_SourceType = pidSource;  
      }
      @Override
      public double pidGet() {
        return Robot.navX.getYaw();
      }
      @Override
      public PIDSourceType getPIDSourceType() {
        return m_SourceType;
      }
    };

    pidOutputGyro = new PIDOutput(){
      @Override
      public void pidWrite(double output) {
        gyroPidOutput = -output; 
      }
    };


    /* Initialize PID Controller Objects */
    PIDRightDistance = new PIDController(pDistance, iDistance, dDistance, pidSourceRight, rightSide);
    PIDLeftDistance = new PIDController(pDistance, iDistance, dDistance, pidSourceLeft, leftSide);
    PIDGyro = new PIDController(pDirection, iDirection, dDirection, pidSourceGyro, pidOutputGyro);

    /* Set the target distances and direction for each thing. Right and left distance are same*/
    PIDRightDistance.setSetpoint(distance);
    PIDLeftDistance.setSetpoint(distance);
    PIDGyro.setSetpoint(direction);

    /* Will set the onTarget boolean to true if it's within the specifed percent margin.
       Percents aren't in decimal form. Ex: Write 15 for 15 percent (not .15)*/
    PIDRightDistance.setPercentTolerance(10);
    PIDLeftDistance.setPercentTolerance(10);
    PIDGyro.setPercentTolerance(10);

    // PIDRightDistance.setOutputRange(-1, 1); 
    // PIDLeftDistance.setOutputRange(-1,1);


  }


  

  @Override
  protected void initialize() {

    /*Reset the encoders of Talons; Reset the navX Gyro*/
    Robot.driveTrainSubsystem.leftRear.setSelectedSensorPosition(0, 0, 0);
    Robot.driveTrainSubsystem.rightFront.setSelectedSensorPosition(0, 0, 0);
    Robot.navX.zeroYaw();

    /*Reset PID Controllers*/
    PIDRightDistance.reset();
    PIDLeftDistance.reset();
    PIDGyro.reset();


    /*Enable the PID Controllers to start running */
    PIDRightDistance.enable(); //encoder tick count 4096
    PIDLeftDistance.enable();
    PIDGyro.enable();

    

    

  }

  @Override
  protected void execute() {


    // System.out.println(Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition(0));
    /*Tank Drive Code */

    double speed = (rightPidOutput + leftPidOutput) / 2 ;
    double wheel = (gyroPidOutput);
    
    tankLeft = speed - wheel;
    tankRight = speed + wheel;





    // System.out.println("Still Running");
    // System.out.println("NavX Reading   :" + Robot.navX.getYaw());
    // System.out.println("RIGHT PID IS ENABLED     " + PIDRightDistance.isEnabled() );
    // System.out.println("Gyro OUTPUT:    " + gyroPidOutput);
    // System.out.println("LEFT OUTPUT:      " + leftPidOutput);
       System.out.println("ANGLE    " + Robot.navX.getYaw());
       System.out.println("Distance                     " + Robot.driveTrainSubsystem.leftRear.getSelectedSensorPosition());
  
    Robot.driveTrainSubsystem.tankDrive(tankLeft , tankRight);
  }


  @Override
  protected boolean isFinished() {
    /*Return true if all these are within their target*/
    return (PIDLeftDistance.onTarget() && PIDRightDistance.onTarget() && PIDGyro.onTarget());
  }

  @Override
  protected void end() {

    /*Disables the PID Controller*/
    PIDRightDistance.disable();
    PIDLeftDistance.disable();
    PIDGyro.disable();

  }


  @Override
  protected void interrupted() {

  }
}
