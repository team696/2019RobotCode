/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import org.frc.team696.robot.commands.ExampleCommand;
import org.frc.team696.robot.subsystems.ExampleSubsystem;
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
    public static OI oi;

    private Command autonomousCommand;
    private SendableChooser<Command> chooser = new SendableChooser<>();

    AHRS ahrs = new AHRS(Port.kMXP, SerialDataType.kProcessedData, (byte)50);

    private static final int k_ticks_per_rev = 2048;
    private static final double k_wheel_diameter = 0.152;
    private static final double k_max_velocity = 2;

    TalonSRX leftRear = new TalonSRX(16);
    TalonSRX leftMid = new TalonSRX(15);
    TalonSRX leftFront = new TalonSRX(14);

    TalonSRX rightRear = new TalonSRX(3);
    TalonSRX rightMid = new TalonSRX(2);
    TalonSRX rightFront = new TalonSRX(1);


    private static final String k_path_name = "Unnamed";

    Trajectory leftTraj;
    Trajectory rightTraj;

    EncoderFollower leftFollower;
    EncoderFollower rightFollower;

    Notifier notifier;

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

        rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);
        leftRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);

        leftRear.setSensorPhase(true);

        // leftRear.setSensorPhase(true);
        rightFront.setSelectedSensorPosition(0);
        leftRear.setSelectedSensorPosition(0);



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

        leftRear.setNeutralMode(NeutralMode.Coast);
        rightFront.setNeutralMode(NeutralMode.Coast);
        ahrs.zeroYaw();
        rightTraj = PathfinderFRC.getTrajectory(k_path_name + ".left");
        leftTraj = PathfinderFRC.getTrajectory(k_path_name + ".right");
    
        leftFollower = new EncoderFollower(leftTraj);
        rightFollower = new EncoderFollower(rightTraj);
        leftFollower.configureEncoder(leftRear.getSelectedSensorPosition(), k_ticks_per_rev, k_wheel_diameter);
        rightFollower.configureEncoder(rightFront.getSelectedSensorPosition(), k_ticks_per_rev, k_wheel_diameter);
        leftFollower.configurePIDVA(1, 0, 0, 1/k_max_velocity, 0);
        rightFollower.configurePIDVA(1, 0, 0, 1/k_max_velocity, 0);
        notifier = new Notifier(this::followPath);
        notifier.startPeriodic(leftTraj.get(0).dt);

    }

    private void followPath(){
        if(leftFollower.isFinished() || rightFollower.isFinished()){
            notifier.stop();
            leftRear.setNeutralMode(NeutralMode.Brake);
            rightFront.setNeutralMode(NeutralMode.Brake);
            leftRear.set(ControlMode.PercentOutput, 0);
            rightFront.set(ControlMode.PercentOutput, 0);
            System.out.println("bye");
        }else{
            double leftSpeed = leftFollower.calculate(leftRear.getSelectedSensorPosition());
            double rightSpeed = rightFollower.calculate(rightFront.getSelectedSensorPosition());
            double heading = ahrs.getYaw();
            double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
            double angleDiff = Pathfinder.boundHalfDegrees(desiredHeading - heading);
            double turn = 0.6 * (-1.0/80.0) * angleDiff;
            double leftDrive = (leftSpeed - turn);
            double rightDrive = -(rightSpeed + turn);
            leftRear.set(ControlMode.PercentOutput, leftDrive);
            rightFront.set(ControlMode.PercentOutput, rightDrive);
            // System.out.println(rightFollower.getSegment().x + " " + rightFollower.getSegment().y);
            // System.out.println(leftSpeed + "   " + rightSpeed + "   " + turn);
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
        // teleop starts running. If you want the autonomous to%
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        ahrs.zeroYaw();
        leftRear.setNeutralMode(NeutralMode.Coast);
        rightFront.setNeutralMode(NeutralMode.Coast);

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        System.out.println(leftRear.getSelectedSensorPosition() + " " + rightFront.getSelectedSensorPosition());
        
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
    }
}
