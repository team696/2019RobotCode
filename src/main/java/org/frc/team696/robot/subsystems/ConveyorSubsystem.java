/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.frc.team696.robot.Robot;
import org.frc.team696.robot.states.ConveyorState;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ConveyorSubsystem extends Subsystem {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_VictorSPX topConveyorMotor;
  public WPI_VictorSPX bottomConveyorMotor;
  public Solenoid conveyorTiltTop;
  public Solenoid conveyorTiltBottom;

  public ConveyorSubsystem(int topConveyorMotorPort, int botomConveyorMotorPort, int conveyorSolPortTop,
      int conveyorSolPortBottom) {
    topConveyorMotor = new WPI_VictorSPX(topConveyorMotorPort);
    bottomConveyorMotor = new WPI_VictorSPX(botomConveyorMotorPort);
    conveyorTiltTop = new Solenoid(17, conveyorSolPortTop);
    conveyorTiltBottom = new Solenoid(17, conveyorSolPortBottom);
  }

  public void runConveyor(double conveyorSpeed) {
    topConveyorMotor.set(conveyorSpeed);
    bottomConveyorMotor.set(-conveyorSpeed);

  }

  public void tiltConveyor(ConveyorState conveyorPos) {
    // case 1 both disabled
    // case 2 bottom enabled
    // case 3 both enabled

    switch (conveyorPos) {
    case MID:
      conveyorTiltBottom.set(true);
      conveyorTiltTop.set(false);
      break;
    case LOW:
      conveyorTiltBottom.set(true);
      conveyorTiltTop.set(true);
      break;
    case HIGH:
      conveyorTiltBottom.set(false);
      conveyorTiltTop.set(false);
      break;

    }

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
