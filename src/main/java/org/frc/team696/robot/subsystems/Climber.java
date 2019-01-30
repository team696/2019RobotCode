/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc.team696.robot.states.ClimberState;
import org.frc.team696.robot.subsystems.ClimberModule;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  ClimberModule fl;
  ClimberModule fr; 
  ClimberModule rl; 
  ClimberModule rr;
  ClimberState state = ClimberState.IDLE;
  

  Climber(TalonSRX fl, TalonSRX fr, TalonSRX rl, TalonSRX rr){
    this.fl = new ClimberModule(fl);
    this.fr = new ClimberModule(fr);
    this.rl = new ClimberModule(rl);
    this.rr = new ClimberModule(rr);
  }

  public ClimberState getState(){
    return this.state;
  }

  /**
   * Setter method for climber subsystem state.
   * This method contains transition checking;
   * that is, if an illegal transition is requested, 
   * it will not actually be implemented.
   * @param newState The requested climber state
   */
  public void setState(ClimberState newState){
    switch(this.state){
      case IDLE:
        switch(newState){
          case IDLE:
            break;
          case MOVE_TO_ARMED:
            this.state = newState;
            break;
          default:
            //Other transitions not allowed
            break;
        }
        break;
      case MOVE_TO_ARMED:
        switch(newState){
          case MOVE_TO_STOWED:
            this.state = newState;
            break;
          default:
            //Other transitions not allowed
            break;
        }
        break;
      case ARMED:
        switch(newState){
          case CLIMBING:
            this.state = newState;
            break;
          case MOVE_TO_STOWED:
            this.state = newState;
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
    // setDefaultCommand(new MySpecialCommand());
  }
}
