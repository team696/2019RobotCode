/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.SuppressWarnings;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.PIDDriveInches;
import frc.robot.commands.PIDDriveRotate;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class OI {
  
  public static Joystick driverController = new Joystick(RobotMap.Controller.DRIVER_CONTROLLER.getValue());
  public static Joystick operatorController = new Joystick(RobotMap.Controller.OPERATOR_CONTROLLER.getValue());
  
  Button B_BUTTON = new JoystickButton(driverController, 2);
  Button A_BUTTON = new JoystickButton(driverController, 1);
  Button Y_BUTTON = new JoystickButton(driverController, 4);

  @SuppressWarnings("resource")
  public OI() {

    A_BUTTON.toggleWhenPressed(new PIDDriveRotate(90));
    B_BUTTON.toggleWhenPressed(new PIDDriveInches(50));
    Y_BUTTON.toggleWhenPressed(new PIDDriveRotate(-90));
    // Ignore this error, no known conflict
    new Notifier(() -> updateOI()).startPeriodic(.1);
  }

  // Anything to be updated should be done in here
  public void updateOI() {
    SmartDashboard.putNumber("robot heading", Robot.getNavXAngle());
    SmartDashboard.putNumber("left encoder pos", Robot.DRIVE_SUBSYSTEM.getLeftRevs());
    SmartDashboard.putNumber("right encoder pos", Robot.DRIVE_SUBSYSTEM.getRightRevs());
  }
}
