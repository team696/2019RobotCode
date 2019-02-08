/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc.team696.robot.subsystems.ClimberModule;

public class ClimberModuleTest extends Command {
  enum TestPhase{
    MovingUp,
    MovingDown
  }

  boolean success = false;
  TestPhase phase = TestPhase.MovingUp;
  boolean finished = false;
  
  ClimberModule module;
  
  private static final double defaultTestMove=0.2;
  private static final double allowablePositionError = 0.01;

  /**
   * Constructor.
   * @param module A reference to the ClimberModule to test
   */
  public ClimberModuleTest(ClimberModule module) {
    // Use requires() here to declare subsystem dependencies
    requires(module);
    this.module = module;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Testing module...");
    this.finished = false;
    this.success = false;
    this.phase = TestPhase.MovingUp;
    //Module must be initialized first
    if(!this.module.isInitialized){
      this.finished = true;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch(this.phase){
      case MovingUp:
        this.module.moveToPosition(defaultTestMove);
        System.out.println(this.module.getCorrectedPosition());
        if(Math.abs(this.module.getCorrectedPosition() - defaultTestMove) < allowablePositionError){
          System.out.println("At position");
          if(!this.module.getRevLimit()){
            this.success = true;
            System.out.println("Success!");
          }
          this.phase = TestPhase.MovingDown;
        }
        break;
      case MovingDown:
          System.out.println(this.module.getCorrectedPosition());
          this.module.moveToPosition(0);
          if(Math.abs(this.module.getCorrectedPosition()) < allowablePositionError || this.module.getRevLimit()){
            if(this.module.getRevLimit()){
              this.module.talon.setSelectedSensorPosition(0);
            }
            this.module.turnOff();
            this.finished = true;
            System.out.println("Done!");
          }
          break;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return this.finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.module.turnOff();
  }
}
