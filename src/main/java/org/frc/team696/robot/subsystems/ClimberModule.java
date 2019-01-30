/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Properties;

/**
 * Add your docs here.
 */
public class ClimberModule extends Subsystem{
  TalonSRX talon;
  public boolean isInitialized = false;
  public Properties properties = new Properties();

  private static final int freePidSlot = 0;
  private static final int climbingPidSlot = 1;

  ClimberModule(TalonSRX talon) {
    // Get properties
    Path configFile = Paths.get(System.getProperty("user.home"), "climber.properties");
    try {
      FileInputStream in = new FileInputStream(configFile.toString());
      try {
        this.properties.load(in);
      } catch (IOException e) {
        System.out.println("Could not read climber properties file. Using defaults instead.");
      }
    } catch (FileNotFoundException e) {
      System.out.println("Could not find climber properties file. Using defaults instead.");
    }

    // Configure talon PID coefficients
    this.talon = talon;
    this.talon.config_kP(freePidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.free.kP", "0.1")));
    this.talon.config_kI(freePidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.free.kI", "0.0")));
    this.talon.config_kD(freePidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.free.kD", "0.0")));
    this.talon.config_kF(freePidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.free.kF", "0.2")));
    this.talon.config_kP(climbingPidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.climbing.kP", "0.1")));
    this.talon.config_kI(climbingPidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.climbing.kI", "0.0")));
    this.talon.config_kD(climbingPidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.climbing.kD", "0.0")));
    this.talon.config_kF(climbingPidSlot, Double.parseDouble(properties.getProperty("climber.coefficients.climbing.kF", "0.5")));

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
      return true;
    } else {
      return false;
    }
  }

  public void moveToPosition(double position){
    moveToPosition(position, freePidSlot);
  }

  public void moveToPosition(double position, int pidSlot){
    this.talon.selectProfileSlot(pidSlot, 0);
    this.talon.set(ControlMode.Position, position);
  }

  public int getPositionError(){
    return this.talon.getClosedLoopError();
  }

  public void turnOff(){
    this.talon.set(ControlMode.PercentOutput, 0);
  }

  public boolean getFwdLimit(){
    return this.talon.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRevLimit(){
    return this.talon.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double getPosition(){
    return this.talon.getSelectedSensorPosition();
  }
}
