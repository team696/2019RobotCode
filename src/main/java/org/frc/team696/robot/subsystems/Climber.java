/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frc.team696.robot.RobotMap;
import org.frc.team696.robot.OI;
import org.frc.team696.robot.commands.ClimberIdle;
import org.frc.team696.robot.commands.ClimberManualControl;
import org.frc.team696.robot.states.ClimberState;
import org.frc.team696.robot.subsystems.ClimberModule;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  public static ClimberModule fl = new ClimberModule("FL Climber Module");
  public static ClimberModule fr = new ClimberModule("FR Climber Module");
  public static ClimberModule rl = new ClimberModule("RL Climber Module");
  public static ClimberModule rr = new ClimberModule("RR Climber Module");
  public static VictorSPX leftPusher = new VictorSPX(RobotMap.leftPusherTalon);
  public static VictorSPX rightPusher = new VictorSPX(RobotMap.rightPusherTalon);

  public static final double frontStagedPosition = 0.175;
  public static final double rearStagedPosition = 0.260;

  // public static final double frontDumbClimbPos = 0.400;
  // public static final double rearDumbClimbPos = 0.320;
  public static final double frontDumbClimbPos = 0.350;
  public static final double rearDumbClimbPos = 0.410;

  public static final double frontLeftClimbPosition = 0.385;
  public static final double frontRightClimbPosition = 0.400;
  public static final double rearLeftClimbPosition = 0.440;
  public static final double rearRightClimbPosition = 0.440;

  // Maximum position error to be considered "armed"
  public static final double armedError = 0.05;

  // Maximum position error to be considered "at height"
  public static final double atHeightError = 0.05;

  // Percent output at which to run the pusher motors
  public static final double pusherPower = 0.5;

  public static final double frontNWOWPower = 0.02;
  public static final double rearNWOWPower = 0.02;

  private static ClimberState state = ClimberState.UNINITIALIZED;

  private static NetworkTableEntry ntflpos;
  private static NetworkTableEntry ntfrpos;
  private static NetworkTableEntry ntrlpos;
  private static NetworkTableEntry ntrrpos;
  private static NetworkTableEntry ntflcurrent;
  private static NetworkTableEntry ntfrcurrent;
  private static NetworkTableEntry ntrlcurrent;
  private static NetworkTableEntry ntrrcurrent;
  private static NetworkTableEntry ntflpercent;
  private static NetworkTableEntry ntfrpercent;
  private static NetworkTableEntry ntrlpercent;
  private static NetworkTableEntry ntrrpercent;

  public Climber() {
    super("Climber");

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable climberTable = inst.getTable("Climber");
    ntflpos = climberTable.getEntry("PosFL");
    ntfrpos = climberTable.getEntry("PosFR");
    ntrlpos = climberTable.getEntry("PosRL");
    ntrrpos = climberTable.getEntry("PosRR");
    ntflcurrent = climberTable.getEntry("CurrentFL");
    ntfrcurrent = climberTable.getEntry("CurrentFR");
    ntrlcurrent = climberTable.getEntry("CurrentRL");
    ntrrcurrent = climberTable.getEntry("CurrentRR");
    ntflpercent = climberTable.getEntry("PercentFL");
    ntfrpercent = climberTable.getEntry("PercentFR");
    ntrlpercent = climberTable.getEntry("PercentRL");
    ntrrpercent = climberTable.getEntry("PercentRR");

    // Create talon objects
    TalonSRX fltalon = new TalonSRX(RobotMap.flClimberTalon);
    TalonSRX frtalon = new TalonSRX(RobotMap.frClimberTalon);
    TalonSRX rltalon = new TalonSRX(RobotMap.rlClimberTalon);
    TalonSRX rrtalon = new TalonSRX(RobotMap.rrClimberTalon);

    // Send talons to modules
    fl.setTalon(fltalon);
    fr.setTalon(frtalon);
    rl.setTalon(rltalon);
    rr.setTalon(rrtalon);

    fl.setInverted(RobotMap.flClimberModuleInverted);
    fl.setSensorPhase(RobotMap.flClimberModuleSensorPhase);

    fr.setInverted(RobotMap.frClimberModuleInverted);
    fr.setSensorPhase(RobotMap.frClimberModuleSensorPhase);

    rl.setInverted(RobotMap.rlClimberModuleInverted);
    rl.setSensorPhase(RobotMap.rlClimberModuleSensorPhase);

    rr.setInverted(RobotMap.rrClimberModuleInverted);
    rr.setSensorPhase(RobotMap.rrClimberModuleSensorPhase);

    leftPusher.setInverted(RobotMap.leftPusherInvert);
    rightPusher.setInverted(RobotMap.rightPusherInvert);
  }

  public static void initialize() {
    if (state == ClimberState.UNINITIALIZED) {
      if (fl.initialize() && fr.initialize() && rl.initialize() && rr.initialize()) {
        state = ClimberState.STOWED;
      }
    }
  }

  public void setState(ClimberState newState) {
    System.out.println("Setting climber state to " + newState);
    state = newState;
  }

  public ClimberState getState() {
    return state;
  }

  /**
   * Manually sets power (percent output) of climber motors.
   * 
   * @param flp Front-left output
   * @param frp Front-right output
   * @param rlp Rear-left output
   * @param rrp Rear-right output
   */
  public static void setPower(double flp, double frp, double rlp, double rrp) {
    fl.setPower(flp);
    fr.setPower(frp);
    rl.setPower(rlp);
    rr.setPower(rrp);
  }

  public static void setPower(double power) {
    setPower(power, power, power, power);
  }

  /**
   * Checks if position control is possible. Convenience function; just an AND of
   * positionControlGood for all the modules
   * 
   * @return If closed-loop control can work
   */
  public boolean getPositionControlGood() {
    // System.out.println("FL position control: " + fl.positionControlGood);
    // System.out.println("FR position control: " + fr.positionControlGood);
    // System.out.println("RL position control: " + rl.positionControlGood);
    // System.out.println("RR position control: " + rr.positionControlGood);
    return (fl.positionControlGood && fr.positionControlGood && rl.positionControlGood && rr.positionControlGood);
  }

  /**
   * Individual (i.e., uncoordinated) move of climber modules. Sends position
   * setpoints to each module.
   * 
   * @param flPos Front-left position setpoint
   * @param frPos Front-right position setpoint
   * @param rlPos Rear-left position setpoint
   * @param rrPos Rear-right position setpoint
   */
  public void moveIndividual(int PIDslot, double flPos, double frPos, double rlPos, double rrPos) {
    fl.moveToPosition(flPos, PIDslot);
    fr.moveToPosition(frPos, PIDslot);
    rl.moveToPosition(rlPos, PIDslot);
    rr.moveToPosition(rrPos, PIDslot);
  }

  public void stowFront() {
    fl.moveToPosition(0);
    fr.moveToPosition(0);
  }

  public void moveIndividual(double flPos, double frPos, double rlPos, double rrPos) {
    moveIndividual(ClimberModule.freePidSlot, flPos, frPos, rlPos, rrPos);
  }

  /**
   * Individual (i.e., uncoordinated) move of climber modules. Sends position
   * setpoints to each module.
   * 
   * @param pos Position setpoint for all modules
   */
  public void moveIndividual(double pos) {
    fl.moveToPosition(pos);
    fr.moveToPosition(pos);
    rl.moveToPosition(pos);
    rr.moveToPosition(pos);
  }

  /**
   * Turns off all modules. Convenience function; calls turnOff() for each module.
   */
  public void turnOff() {
    fl.turnOff();
    fr.turnOff();
    rl.turnOff();
    rr.turnOff();
  }

  /**
   * Gets the maximum position error among the 4 modules.
   * 
   * @return The value of the greatest position error
   */
  public double getMaximumPositionError() {
    double flerror = fl.getCorrectedPositionError();
    double frerror = fr.getCorrectedPositionError();
    double rlerror = rl.getCorrectedPositionError();
    double rrerror = rr.getCorrectedPositionError();

    return Math.max(flerror, Math.max(frerror, Math.max(rlerror, rrerror)));
  }

  public void climberPeriodic() {
    ntflpos.setDouble(fl.getCorrectedPosition());
    ntfrpos.setDouble(fr.getCorrectedPosition());
    ntrlpos.setDouble(rl.getCorrectedPosition());
    ntrrpos.setDouble(rr.getCorrectedPosition());
    ntflcurrent.setDouble(fl.talon.getOutputCurrent());
    ntfrcurrent.setDouble(fr.talon.getOutputCurrent());
    ntrlcurrent.setDouble(rl.talon.getOutputCurrent());
    ntrrcurrent.setDouble(rr.talon.getOutputCurrent());
    ntflpercent.setDouble(fl.talon.getMotorOutputPercent());
    ntfrpercent.setDouble(fr.talon.getMotorOutputPercent());
    ntrlpercent.setDouble(rl.talon.getMotorOutputPercent());
    ntrrpercent.setDouble(rr.talon.getMotorOutputPercent());

    if ( getPositionControlGood()) {
      // Service climber state
      switch (state) {
      case UNINITIALIZED:
        break;
      case STOWED:
        moveIndividual(0.0);
        break;
      case MOVE_TO_ARMED:
        if (getMaximumPositionError() < armedError) {
          setState(ClimberState.ARMED);
        }
      case ARMED:
        moveIndividual(frontStagedPosition, frontStagedPosition, rearStagedPosition, rearStagedPosition);
        break;
      case CLIMBING:
        moveIndividual(frontLeftClimbPosition, frontRightClimbPosition, rearLeftClimbPosition, rearRightClimbPosition);
        if (getMaximumPositionError() < atHeightError) {
          setState(ClimberState.AT_HEIGHT);
        }
        break;
      case AT_HEIGHT:
        moveIndividual(frontLeftClimbPosition, frontRightClimbPosition, rearLeftClimbPosition, rearRightClimbPosition);
        setPusherPower(pusherPower);
        if (!frontWOW()) {
          // If no weight on front wheels, retract
          setState(ClimberState.FRONT_ON_PLATFORM);
          stowFront();
        }
        break;
      case FRONT_ON_PLATFORM:
        stowFront();
        if (!rearWOW()) {
          moveIndividual(0);
          setState(ClimberState.STOWED);
        }
        break;
      }
    }
    else{
        turnOff();
    }
  }

  public void setPusherPower(double power) {
    leftPusher.set(ControlMode.PercentOutput, power);
    rightPusher.set(ControlMode.PercentOutput, power);
  }

  public boolean frontWOW() {
    if (fl.talon.getMotorOutputPercent() < frontNWOWPower && fr.talon.getMotorOutputPercent() < frontNWOWPower) {
      return false;
    } else {
      return true;
    }
  }

  public boolean rearWOW() {
    if (rl.talon.getMotorOutputPercent() < rearNWOWPower && rr.talon.getMotorOutputPercent() < rearNWOWPower) {
      return false;
    } else {
      return true;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new ClimberIdle());
  }
}
