/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import org.frc.team696.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoAlignment extends CommandGroup {
  /**
   * Add your docs here.
   */

  private double encoder_ticks = 3000;
  private double halfBotEncoderTicks = 3488;
  private double hopstAngle = 28.264;
  private double angle;

  public AutoAlignment(double angle, double error) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new ZeroYaw());
    addSequential(new DriveBack(halfBotEncoderTicks + error));
    if(angle<0){
    addSequential(new DriveToAngleCommand(90+angle),1);
    }
    else{
    addSequential(new DriveToAngleCommand(-(90-angle)/*(180 - (90 + angle)))*/),1);
    }
    addSequential(new DriveForward(((halfBotEncoderTicks * Math.abs(Math.sin(Math.toRadians(angle))) + halfBotEncoderTicks/2)))); 
    addSequential(new DriveToAngleCommand(angle), 1);
    addSequential(new AutoAlignOff());
  }

}
