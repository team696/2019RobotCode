/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc.team696.robot.commands.TiltCommand;
import org.frc.team696.robot.subsystems.DriveTrainSubsystem;
import org.frc.team696.robot.subsystems.IntakeSubsystem;
import org.frc.team696.robot.subsystems.TiltSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
// If you rename or move this class, update the build.properties file in the project root
public class Robot extends TimedRobot {

   
    public static OI oi;
public static IntakeSubsystem intake = new IntakeSubsystem(RobotMap.intakeAPort, RobotMap.intakeBPort, RobotMap.solPort);
public static DriveTrainSubsystem drive = new DriveTrainSubsystem(RobotMap.leftFrontPort, RobotMap.leftMidPort, RobotMap.leftRearPort, RobotMap.rightFrontPort, RobotMap.rightMidPort, RobotMap.rightRearPort);
public static TiltSubsystem tilt = new TiltSubsystem(RobotMap.solTiltBoo);
//public static TiltCommand tiltCommand = new TiltCommand(solBool);
double speed;
double turn;
double leftSpeed;
double rightSpeed; 
double speedFactor = 0.75;

    private Command autonomousCommand;
    private SendableChooser<Command> chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        oi = new OI();;
        SmartDashboard.putData("Auto mode", chooser);
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

    speed =  OI.stick.getRawAxis(1);
    turn = OI.wheel.getRawAxis(0);

      
    leftSpeed = speed + turn;
    rightSpeed = speed - turn;

    if (OI.controlPanel.getRawButton(2)){
        speedFactor = 0.75;
    }

    if (OI.controlPanel.getRawButton(1)){
        speedFactor = 0.5;
    }

    if (OI.controlPanel.getRawButton(7)){
        intake.runIntake(0.5);
    }

   else if (OI.controlPanel.getRawButton(8)){
        intake.runIntake(-0.5);
    }
        else{intake.runIntake(0);
        }

    if (OI.controlPanel.getRawButton(9)){
        intake.solIntake(true);
    }
        else intake.solIntake(false);


    OI.tilt.toggleWhenPressed(new TiltCommand(true));
    OI.tilt.toggleWhenPressed(new TiltCommand(false));
    

  drive.execute(speedFactor*leftSpeed, speedFactor*rightSpeed);

    // intake.solIntake(true);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
    }
}
