/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc.team696.robot.RobotMap;
/**
 * Add your docs here.
 */
public class ClimberModule extends Subsystem{
  public TalonSRX talon;
  public boolean isInitialized = false;
  public boolean positionControlGood = false;

  public static final int freePidSlot = 0;
  public static final int climbingPidSlot = 1;

  private static final double encoderTicksPerRev = 4096.0*3.0;

  private static final double freePidkP = 0.6;
  private static final double climbingPidkP = 0.8;
  private static final double freeMaxOutput = 1.0;
  private static final double climbingMaxOutput = 0.5; 

  public ClimberModule(String name){
    super(name);
    //TODO: set up logging
  }


  /**
   * Sets the talon reference for a module.
   * Also configures the talon: PID coefficients, limit switches, etc.
   * @param talon An existing reference to the talon on this module
   */
  public void setTalon(TalonSRX talon) {
    // Configure talon PID coefficients
    this.talon = talon;
    this.talon.config_kP(freePidSlot, freePidkP);
    this.talon.config_kI(freePidSlot, 0.0);
    this.talon.config_kD(freePidSlot, 0.0);
    this.talon.config_kF(freePidSlot, 0.0);
    this.talon.configClosedLoopPeakOutput(freePidSlot, freeMaxOutput);

    this.talon.config_kP(climbingPidSlot, climbingPidkP);
    this.talon.config_kI(climbingPidSlot, 0.0);
    this.talon.config_kD(climbingPidSlot, 0.0);
    this.talon.config_kF(climbingPidSlot, 0.15);
    this.talon.configClosedLoopPeakOutput(climbingPidSlot, climbingMaxOutput);

    this.talon.configPeakOutputForward(1);
    this.talon.configPeakOutputReverse(-1);
    this.talon.setNeutralMode(NeutralMode.Brake);
    this.talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //this.talon.setSensorPhase(false);
    //this.talon.setInverted(true);

    this.talon.selectProfileSlot(freePidSlot, 0);
  }

  @Override
  public void initDefaultCommand(){
    
  }

  /**
   * Initializes the climber module.
   * Checks if the reverse limit switch is closed.
   * If so, zeros the encoder and sets the isInitialized attribute.
   * @return True if initialization was successful, false otherwise. 
   */
  public boolean initialize() {
    // Check that limit switches indicate stowed position
    if (this.talon.getSensorCollection().isRevLimitSwitchClosed()) {
      this.talon.setSelectedSensorPosition(0);
      this.isInitialized = true;
      System.out.println(this.getName() + " initialized");
      return true;
    } else {
      System.out.println(this.getName() + " not initialized!");
      return false;
    }
  }

  public void moveToPosition(double position){
    moveToPosition(position, freePidSlot);
  }

  public void moveToPosition(double position, int pidSlot){
    //System.out.println("Climber "+this.getName()+" moving to "+position);
    this.talon.selectProfileSlot(pidSlot, 0);
    this.talon.set(ControlMode.Position, this.convertSetpoint(position));
  }

  public double getCorrectedPositionError(){
    return nativeUnitsToRevolutions(this.talon.getClosedLoopError());
  }

  public void turnOff(){
    this.talon.set(ControlMode.PercentOutput, 0);
  }

  public void setPower(double power){
    this.talon.set(ControlMode.PercentOutput, power);
  }

  public boolean getFwdLimit(){
    return this.talon.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRevLimit(){
    return this.talon.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double nativeUnitsToRevolutions(int encoderTicks){
    return ((double)encoderTicks)/encoderTicksPerRev;
  }

  public double getCorrectedPosition(){
    return nativeUnitsToRevolutions(this.talon.getSelectedSensorPosition());
  }

  public int convertSetpoint(double revs){
    return (int)(revs*encoderTicksPerRev);
  }

  public void setSensorPhase(boolean phase){
    System.out.println("Setting phase to "+phase+" on "+this.getName());
    this.talon.setSensorPhase(phase);
  }

  public void setInverted(boolean inverted){
    System.out.println("Setting inverted to "+inverted+" on "+this.getName());
    this.talon.setInverted(inverted);
  }

  /**
   * Converts a velocity in rps to encoder ticks / 100 ms.
   * TrajectoryPoints require target velocity in sensor ticks / 100 ms.
   * This is a helper function to convert from rps, taking into account
   * sensor resolution and gearing. 
   * @param rps Revolutions per second of arm
   * @return Encoder ticks per 100 ms
   */
  public int convertVelocityForProfile(double rps){
    rps *= encoderTicksPerRev; //Now is encoder ticks per second
    rps /= 10; //Now is encoder ticks per 100 ms
    return (int) rps;
  }

}
