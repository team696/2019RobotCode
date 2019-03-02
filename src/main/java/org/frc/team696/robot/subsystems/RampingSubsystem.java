/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import org.frc.team696.robot.Constants;
import org.frc.team696.robot.OI;
import org.frc.team696.robot.states.ConveyorState;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class RampingSubsystem extends Subsystem {

  ConveyorState state;

  public double speed;
  public double wheel;

  private double highRampDownRate = 0.03;
  private double midRampDownRate = 0.05;
  private double lowRampDownRate = 0.07;

  private double highRampUpRate = 0.03;
  private double midRampUpRate = 0.05;
  private double lowRampUpRate = 0.07;

  private double commandedSpeed;

  // state booleans

  private boolean rampUpHigh;
  private boolean rampUpMid;
  private boolean rampUpLow;
  private boolean rampDownHigh;
  private boolean rampDownMid;
  private boolean rampDownLow;

  public void ramp(ConveyorState state) {

    this.state = state;

    switch (state) {

    case HIGH:
      rampUpHigh = true;
      rampUpMid = false;
      rampUpLow = false;
      rampDownHigh = true;
      rampDownMid = false;
      rampDownLow = false;
      break;

    case MID:
      rampUpHigh = false;
      rampUpMid = true;
      rampUpLow = false;
      rampDownHigh = false;
      rampDownMid = true;
      rampDownLow = false;
      break;

    case LOW:
      rampUpHigh = false;
      rampUpMid = false;
      rampUpLow = true;
      rampDownHigh = false;
      rampDownMid = false;
      rampDownLow = true;
      break;

    }

    // if (rampUpHigh) {
    //   rampUpHigh();
    // }

    // if (rampUpMid) {
    //   rampUpMid();
    // }

    // if (rampUpLow) {
    //   rampUpLow();
    // }

    if (rampDownHigh) {
      rampDownHigh();
    }

    if (rampDownMid) {
      rampDownMid();
    }

    if (rampDownLow) {
      rampDownLow();
    }

  }

  // private void rampUpHigh() {
  //   System.out.println("Running rampUpHigh");
  //   commandedSpeed = OI.wheel.getRawAxis(0);
  //   wheel = OI.xboxController.getRawAxis(1);

  //   if (speed > commandedSpeed && commandedSpeed < 0) {
  //     speed -= highRampRate;
  //   } else {
  //     speed = commandedSpeed;
  //   }

  // }

  // private void rampUpMid() {
  //   System.out.println("Running rampUpMid");
  //   commandedSpeed = OI.wheel.getRawAxis(0);
  //   wheel = OI.xboxController.getRawAxis(1);

  // }

  // private void rampUpLow() {
  //   System.out.println("Running rampUpLow");
  //   commandedSpeed = OI.wheel.getRawAxis(0);
  //   wheel = OI.xboxController.getRawAxis(1);

  // }

  private void rampDownHigh() {
    System.out.println("Running rampDownHigh");
    wheel = -OI.wheel.getRawAxis(0);
    commandedSpeed = OI.xboxController.getRawAxis(1);

    if (speed < commandedSpeed) {
      speed += highRampUpRate;
      System.out.println("if");
    }else if(speed > commandedSpeed){
      speed -= highRampDownRate;
      System.out.println("else if");
    }else{
      speed = commandedSpeed;
      System.out.println("else");
    }

  }

  private void rampDownMid() {
    System.out.println("Running rampDownMid");
    wheel = -OI.wheel.getRawAxis(0);
    commandedSpeed = OI.xboxController.getRawAxis(1);

    if (speed < commandedSpeed) {
      speed += midRampUpRate;
      System.out.println("if");
    }else if(speed > commandedSpeed){
      speed -= midRampDownRate;
      System.out.println("else if");
    }else{
      speed = commandedSpeed;
      System.out.println("else");
    }

  }

  private void rampDownLow() {
    System.out.println("Running rampDownLow");
    wheel = -OI.wheel.getRawAxis(0);
    commandedSpeed = OI.xboxController.getRawAxis(1);

    if (speed < commandedSpeed) {
      speed += lowRampUpRate;
      System.out.println("if");
    }else if(speed > commandedSpeed){
      speed -= lowRampDownRate;
      System.out.println("else if");
    }else{
      speed = commandedSpeed;
      System.out.println("else");
    }

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
