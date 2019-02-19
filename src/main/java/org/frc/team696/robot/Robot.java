/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc.team696.robot.commands.ExampleCommand;
import org.frc.team696.robot.subsystems.Climber;
import org.frc.team696.robot.subsystems.ClimberModule;
import org.frc.team696.robot.subsystems.ExampleSubsystem;
import org.frc.team696.robot.RobotMap;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
// If you rename or move this class, update the build.properties file in the project root
public class Robot extends TimedRobot {

    public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    public static final Climber climber = new Climber();
    //public static final Climber climber = null;
    //public ClimberModule testModule = new ClimberModule("Test Module");
    public static OI oi;
    private Command autonomousCommand;
    private SendableChooser<Command> chooser = new SendableChooser<>();

    public Compressor comp = new Compressor();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        oi = new OI();
        chooser.addDefault("Default Auto", new ExampleCommand());
        // chooser.addObject("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", chooser);
        //this.testModule.setTalon(testTalon);
        //this.testModule.setInverted(RobotMap.flClimberModuleInverted);
        //this.testModule.setSensorPhase(RobotMap.flClimberModuleSensorPhase);
        //this.singlemodule.setTalon(fl);
        //this.singlemodule.initialize();
        Climber.initialize();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        //System.out.println(Climber.fr.getCorrectedPosition());
        //Climber.fl.talon.setSensorPhase(false);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.start();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        

        Scheduler.getInstance().run();
        Climber.leftPusher.set(ControlMode.PercentOutput, -OI.operatorPanel.getRawAxis(3));
        Climber.rightPusher.set(ControlMode.PercentOutput, -OI.operatorPanel.getRawAxis(3));
        //System.out.println(OI.operatorPanel.getRawAxis(3));
        if(!comp.enabled()){
            comp.start();
            System.out.println("hello hopst");
        }
        System.out.println(comp.enabled());
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
    }

    @Override
    public void robotPeriodic(){
        //System.out.println("Climber position:" + this.testModule.getCorrectedPosition());
        climber.climberPeriodic();
        //System.out.println(Climber.fr.getName() + " is at "+Climber.fr.getCorrectedPosition());
        //System.out.println(Climber.rl.getName() + " is outputting "+Climber.fr.talon.getMotorOutputPercent());

    }
}
