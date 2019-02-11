/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frc.team696.robot.RobotMap;
import org.frc.team696.robot.OI;
import org.frc.team696.robot.commands.ClimberIdle;
import org.frc.team696.robot.commands.ClimberManualControl;
import org.frc.team696.robot.subsystems.ClimberModule;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  public static ClimberModule fl = new ClimberModule("FL Climber Module");
  public static ClimberModule fr = new ClimberModule("FR Climber Module"); 
  //public static ClimberModule rl = new ClimberModule("RL Climber Module"); 
  //public static ClimberModule rr = new ClimberModule("RR Climber Module");
  
  public static boolean isInitialized = false;

  private static NetworkTableEntry ntflpos;
  private static NetworkTableEntry ntfrpos;
  private static NetworkTableEntry ntrlpos;
  private static NetworkTableEntry ntrrpos;

  public Climber(){
    super("Climber");

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable climberTable = inst.getTable("climber");
    ntflpos = climberTable.getEntry("PosFL");
    ntfrpos = climberTable.getEntry("PosFR");
    ntrlpos = climberTable.getEntry("PosRL");
    ntrrpos = climberTable.getEntry("PosRR");

    //Create talon objects
    TalonSRX fltalon = new TalonSRX(RobotMap.flClimberTalon);
    TalonSRX frtalon = new TalonSRX(RobotMap.frClimberTalon);
    //TalonSRX rltalon = new TalonSRX(RobotMap.rlClimberTalon);
    //TalonSRX rrtalon = new TalonSRX(RobotMap.rrClimberTalon);

    //Send talons to modules
    fl.setTalon(fltalon);
    fr.setTalon(frtalon);
    //rl.setTalon(rltalon);
    //rr.setTalon(rrtalon);

    fl.setInverted(RobotMap.flClimberModuleInverted);
    fl.setSensorPhase(RobotMap.flClimberModuleSensorPhase);

    fr.setInverted(RobotMap.frClimberModuleInverted);
    fr.setSensorPhase(RobotMap.frClimberModuleSensorPhase);

    //rl.setInverted(RobotMap.rlClimberModuleInverted);
    //rl.setSensorPhase(RobotMap.rlClimberModuleSensorPhase);

    //rr.setInverted(RobotMap.rrClimberModuleInverted);
    //rr.setSensorPhase(RobotMap.rrClimberModuleSensorPhase);
  }

  public static void initialize(){
    isInitialized = (fl.initialize() && fr.initialize());
  }

  /**
   * Manually sets power (percent output) of climber motors.
   * 
   * @param flp Front-left output
   * @param frp Front-right output
   * @param rlp Rear-left output
   * @param rrp Rear-right output
   */
  public static void setPower(double flp, double frp, double rlp, double rrp){
    fl.setPower(flp);
    fr.setPower(frp);
    //rl.setPower(rlp);
    //rr.setPower(rrp);
  }

  public static void setPower(double power){
    setPower(power, power, power, power);
  }

  /**
   * Checks if position control is possible.
   * Convenience function; just an AND of positionControlGood for all the modules
   * @return If closed-loop control can work
   */
  public boolean getPositionControlGood(){
    //return (fl.positionControlGood && fr.positionControlGood && rl.positionControlGood && rr.positionControlGood);
    return fl.positionControlGood;
  }

  /**
   * Individual (i.e., uncoordinated) move of climber modules.
   * Sends position setpoints to each module. 
   * @param flPos Front-left position setpoint
   * @param frPos Front-right position setpoint
   * @param rlPos Rear-left position setpoint
   * @param rrPos Rear-right position setpoint
   */
  public void moveIndividual(double flPos, double frPos, double rlPos, double rrPos){
    fl.moveToPosition(flPos);
    fr.moveToPosition(frPos);
    //rl.moveToPosition(rlPos);
    //rr.moveToPosition(rrPos);
  }

  /**
   * Individual (i.e., uncoordinated) move of climber modules.
   * Sends position setpoints to each module. 
   * @param pos Position setpoint for all modules
   */
  public void moveIndividual(double pos){
    fl.moveToPosition(pos);
    fr.moveToPosition(pos);
    //rl.moveToPosition(pos);
    //rr.moveToPosition(pos);
  }

  /**
   * Turns off all modules.
   * Convenience function; calls turnOff() for each module.
   */
  public void turnOff(){
    fl.turnOff();
    fr.turnOff();
    //rl.turnOff();
    //rr.turnOff();
  }

  public void climberPeriodic(){
    ntflpos.setDouble(fl.getCorrectedPosition());
    ntfrpos.setDouble(fr.getCorrectedPosition());
    //ntrlpos.setDouble(rl.getCorrectedPosition());
    //ntrrpos.setDouble(rr.getCorrectedPosition());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberIdle());
  }
}
