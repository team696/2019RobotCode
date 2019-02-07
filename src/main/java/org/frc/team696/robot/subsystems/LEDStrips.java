/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.frc.team696.robot.commands.LEDStripsSetPattern;

import edu.wpi.first.wpilibj.SPI;
/**
 * Add your docs here.
 */
public class LEDStrips extends Subsystem {
  public SPI spi;

  public void init(){
    this.spi = new SPI(SPI.Port.kOnboardCS0);
    this.spi.setClockRate(10000);
  }

  public void SetPattern(int pattern){
    byte[] toSend = {0};
    toSend[0] = (byte)(pattern & 0xff);
    //toSend[1] = (byte)(~pattern & 0xff);
    this.spi.write(toSend, 1);
    toSend[0] = (byte)(~pattern & 0xff);
    //toSend[0] = (byte)(0x0f);
    this.spi.write(toSend, 1);
    
    //System.out.printf("%h, %h\n", toSend[0], toSend[1]);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new LEDStripsSetPattern(0));
  }
}
