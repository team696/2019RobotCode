/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.frc.team696.robot.Robot;
import org.frc.team696.robot.commands.ClimberDumbClimb;
import org.frc.team696.robot.commands.ClimberInit;
import org.frc.team696.robot.commands.ClimberManualControl;
import org.frc.team696.robot.commands.ClimberModuleTest;
import org.frc.team696.robot.commands.ClimberSemiAuto;
import org.frc.team696.robot.commands.ClimberSetState;
import org.frc.team696.robot.commands.ClimberTest;
import org.frc.team696.robot.commands.ConveyorCommand;
import org.frc.team696.robot.states.ClimberState;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public static Joystick xboxController = new Joystick(0);
    public static Joystick wheel = new Joystick(1);

    public static Button conveyorButton = new JoystickButton(xboxController, 2);
    public static Button button1 = new JoystickButton(xboxController, 1);
    public static Button button7 = new JoystickButton(xboxController, 7);
    public static Button button8 = new JoystickButton(xboxController, 8);

    //public static Button climberManualSwitch = new JoystickButton(operatorPanel, 5);
    public static Button climberModuleTest = new JoystickButton(xboxController, 5);
    //public static Button climberStage = new JoystickButton(operatorPanel, 7);
    public static Button dumbClimb = new JoystickButton(xboxController, 6);
    //public static Button semiAutoClimb = new JoystickButton(operatorPanel, 6);
    //public static Button reinit = new JoystickButton(operatorPanel, 2);

    public OI(){
        //climberManualSwitch.whileHeld(new ClimberManualControl());
        climberModuleTest.whenPressed(new ClimberTest());
        //climberManualSwitch.whileHeld(new ClimberManualControl());
        //climberStage.whenPressed(new ClimberSetState(ClimberState.MOVE_TO_ARMED));
        //climberStage.whenReleased(new ClimberSetState(ClimberState.STOWED));
        dumbClimb.whenPressed(new ClimberSetState(ClimberState.CLIMBING));
        dumbClimb.whenReleased(new ClimberSetState(ClimberState.STOWED));
        //semiAutoClimb.whileHeld(new ClimberSemiAuto());
        //reinit.whenPressed(new ClimberInit());
        OI.button7.whenPressed(new ConveyorCommand(1));
        OI.button7.whenReleased(new ConveyorCommand(0));

        OI.button8.whenPressed(new ConveyorCommand(-1));
        OI.button8.whenReleased(new ConveyorCommand(0));
        
    }

    public static double getClimberManual(){
        return xboxController.getRawAxis(1);
    }
    // CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    // joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());



}
