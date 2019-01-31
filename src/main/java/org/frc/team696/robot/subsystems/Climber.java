/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc.team696.robot.OI;
import org.frc.team696.robot.commands.ClimberManualControl;
import org.frc.team696.robot.states.ClimberState;
import org.frc.team696.robot.subsystems.ClimberModule;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  private static ClimberModule fl;
  private static ClimberModule fr; 
  private static ClimberModule rl; 
  private static ClimberModule rr;
  private static ClimberState state = ClimberState.IDLE;
  
  public static boolean isManual = false;

  /**
   * Set up ClimberModule instances with preexisting TalonSRXs.
   * 
   * @param fltalon Front-left TalonSRX
   * @param frtalon Front-right TalonSRX
   * @param rltalon Rear-left TalonSRX
   * @param rrtalon Rear-right TalonSRX
   */
  public static void setControllers(TalonSRX fltalon, TalonSRX frtalon, TalonSRX rltalon, TalonSRX rrtalon){
    fl = new ClimberModule(fltalon);
    fr = new ClimberModule(frtalon);
    rl = new ClimberModule(rltalon);
    rr = new ClimberModule(rrtalon);
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
    rl.setPower(rlp);
    rr.setPower(rrp);
  }

  public static void setPower(double power){
    setPower(power, power, power, power);
  }

  public ClimberState getState(){
    return state;
  }

  /**
   * Setter method for climber subsystem state.
   * This method contains transition checking;
   * that is, if an illegal transition is requested, 
   * it will not actually be implemented.
   * @param newState The requested climber state
   */
  public static void setState(ClimberState newState){
    switch(state){
      case IDLE:
        switch(newState){
          case IDLE:
            break;
          case MOVE_TO_ARMED:
            state = newState;
            break;
          default:
            //Other transitions not allowed
            break;
        }
        break;
      case MOVE_TO_ARMED:
        switch(newState){
          case MOVE_TO_STOWED:
            state = newState;
            break;
          default:
            //Other transitions not allowed
            break;
        }
        break;
      case ARMED:
        switch(newState){
          case CLIMBING:
            state = newState;
            break;
          case MOVE_TO_STOWED:
            state = newState;
            break;
          default:
            //Other transitions not allowed
            break;
        }
        break;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberManualControl());
  }
}
