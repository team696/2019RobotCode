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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.frc.team696.robot.RobotMap;
import org.frc.team696.robot.OI;
import org.frc.team696.robot.Robot;
import org.frc.team696.robot.states.ClimberState;
import org.frc.team696.robot.states.ConveyorState;
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
  public static final double frontRightClimbPosition = 0.410;
  public static final double rearLeftClimbPosition = 0.440;
  public static final double rearRightClimbPosition = 0.450;

  // Maximum position error to be considered "armed"
  public static final double armedError = 0.05;

  // Maximum position error to be considered "at height"
  public static final double atHeightError = 0.05;

  // Percent output at which to run the pusher motors
  public static final double pusherPower = 1.0;

  public static final double frontNWOWPower = 0.02;
  public static final double rearNWOWPower = 0.02;

  private static ClimberState state = ClimberState.UNINITIALIZED;
  public static boolean isManualControl = false;

  // private static NetworkTableEntry ntflpos;
  // private static NetworkTableEntry ntfrpos;
  // private static NetworkTableEntry ntrlpos;
  // private static NetworkTableEntry ntrrpos;
  // private static NetworkTableEntry ntflcurrent;
  // private static NetworkTableEntry ntfrcurrent;
  // private static NetworkTableEntry ntrlcurrent;
  // private static NetworkTableEntry ntrrcurrent;
  // private static NetworkTableEntry ntflpercent;
  // private static NetworkTableEntry ntfrpercent;
  // private static NetworkTableEntry ntrlpercent;
  // private static NetworkTableEntry ntrrpercent;

  /**
   * Climber subsystem; responsible for coordinating the actions of the climber
   * modules.
   */
  public Climber() {
    super("Climber");

    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // NetworkTable climberTable = inst.getTable("Climber");
    // ntflpos = climberTable.getEntry("PosFL");
    // ntfrpos = climberTable.getEntry("PosFR");
    // ntrlpos = climberTable.getEntry("PosRL");
    // ntrrpos = climberTable.getEntry("PosRR");
    // ntflcurrent = climberTable.getEntry("CurrentFL");
    // ntfrcurrent = climberTable.getEntry("CurrentFR");
    // ntrlcurrent = climberTable.getEntry("CurrentRL");
    // ntrrcurrent = climberTable.getEntry("CurrentRR");
    // ntflpercent = climberTable.getEntry("PercentFL");
    // ntfrpercent = climberTable.getEntry("PercentFR");
    // ntrlpercent = climberTable.getEntry("PercentRL");
    // ntrrpercent = climberTable.getEntry("PercentRR");

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

  /**
   * Convenience method; calls initialize on each of the climber modules. If all
   * initializations are successful, transitions state to stowed.
   */
  public void initialize() {
    // if (state == ClimberState.UNINITIALIZED) {
    //   if (fl.initialize() && fr.initialize() && rl.initialize() && rr.initialize()) {
    //     setState(ClimberState.STOWED);
    //   }
    // }
    fl.initialize();
    fr.initialize();
    rl.initialize();
    rr.initialize();
    setState(ClimberState.STOWED);
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

  public void stowFront() {
    fl.moveToPosition(0);
    fr.moveToPosition(0);
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

  /**
   * Periodic climber tasks. Publishes data to Network Tables & services climber
   * state machine. This should be called from RobotPeriodic() or similar.
   */
  public void climberPeriodic() {

    //Implement pusher control
    if(OI.pusherOverride.get() || state == ClimberState.AT_HEIGHT || state == ClimberState.FRONT_ON_PLATFORM){
      setPusherPower(pusherPower);
      OI.operatorPanel.setOutput(1, true);
    }
    else{
      setPusherPower(0);
      OI.operatorPanel.setOutput(1, false);
    }

    if (getPositionControlGood() && !isManualControl) {
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
        OI.operatorPanel.setOutput(2, true);
        break;

      case CLIMBING:
        if(Robot.conveyorState == ConveyorState.HIGH){
          moveIndividual(frontLeftClimbPosition, frontRightClimbPosition, rearLeftClimbPosition, rearRightClimbPosition);
        }else{
          System.out.println("Conveyor not in high position!");
        }
        if (getMaximumPositionError() < atHeightError) {
          setState(ClimberState.AT_HEIGHT);
        }
        OI.operatorPanel.setOutput(2, true);
        break;

      case AT_HEIGHT:
        moveIndividual(frontLeftClimbPosition, frontRightClimbPosition, rearLeftClimbPosition, rearRightClimbPosition);
        //if (!frontWOW()) {
          // If no weight on front wheels, retract
          //setState(ClimberState.FRONT_ON_PLATFORM);
          //stowFront();
        //}
        OI.operatorPanel.setOutput(2, true);
        break;

      case FRONT_ON_PLATFORM:
        stowFront();
        if (!rearWOW()) {
          moveIndividual(0);
          setState(ClimberState.STOWED);
        }
        OI.operatorPanel.setOutput(2, true);
        break;
      case HOLD:
        turnOff();
        break;

      default:
        // Should never happen
        setState(ClimberState.UNINITIALIZED);
        break;
      }
    } else {
      turnOff();
    }
  }

  public void setPusherPower(double power) {
    leftPusher.set(ControlMode.PercentOutput, power);
    rightPusher.set(ControlMode.PercentOutput, power);
  }

  /**
   * WIP: Do not use for the time being. Intended to determine if the front arms
   * are supporting any weight.
   * 
   * @return True if there is weight on the front arms.
   */
  public boolean frontWOW() {
    if (fl.talon.getMotorOutputPercent() < frontNWOWPower && fr.talon.getMotorOutputPercent() < frontNWOWPower) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * WIP: Do not use for the time being. Intended to determine if the rear arms
   * are supporting any weight.
   * 
   * @return True if there is weight on the rear arms.
   */
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
    // setDefaultCommand(new ClimberIdle());
  }
}
