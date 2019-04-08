/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.frc.team696.robot.subsystems.DriveTrainSubsystem;
import org.frc.team696.robot.subsystems.RampingSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.frc.team696.robot.commands.ConveyorCommand;
import org.frc.team696.robot.states.ConveyorState;
import org.frc.team696.robot.subsystems.ConveyorSubsystem;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc.team696.robot.subsystems.Climber;
import org.frc.team696.robot.subsystems.ClimberModule;
import org.frc.team696.robot.RobotMap;
import org.frc.team696.robot.subsystems.ConveyorSubsystem;
import org.frc.team696.robot.subsystems.DriveTrainSubsystem;
import org.frc.team696.robot.subsystems.HatchSubsystem;
import org.frc.team696.robot.subsystems.RampingSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
// If you rename or move this class, update the build.properties file in the
// project root
public class Robot extends TimedRobot {

    public static final Climber climber = new Climber();
    // public static final Climber climber = null;
    // public ClimberModule testModule = new ClimberModule("Test Module");
    public static OI oi;
    public static ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem(RobotMap.topConveyorMotorPort,
            RobotMap.bottomConveyorMotorPort, RobotMap.conveyorSolPortTop, RobotMap.conveyorSolPortBottom);
    public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.leftFrontPort,
            RobotMap.leftMidPort, RobotMap.leftRearPort, RobotMap.rightRearPort, RobotMap.rightMidPort,
            RobotMap.rightFrontPort);
    public static HatchSubsystem hatchSubsystem = new HatchSubsystem(RobotMap.hatchActuatorPort, RobotMap.hatchPositionPort);
    public static RampingSubsystem rampingSubsystem = new RampingSubsystem();
    private Command autonomousCommand;
    private SendableChooser<Command> chooser = new SendableChooser<>();

    public static int conveyorTiltCase;
    public static ConveyorState conveyorState = ConveyorState.HIGH;

    public Compressor comp = new Compressor(17);

    public double stick;
    public double wheel;

    public double leftSpeed;
    public double rightSpeed;

    double a = 0.170641; // 0.286095 0.170641
    double h = -0.258475; // -0.243151 -0.258475
    double k = 0.364407; // -0.130137 0.364407

    double speedTurnScale;

    Timer autoTimer = new Timer();
    double bookitTime = 1.7;

    PowerDistributionPanel pdp = new PowerDistributionPanel(18);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        oi = new OI();
        // chooser.addObject("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", chooser);
        comp.start();
        climber.initialize();
    }

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        // System.out.println(Climber.fr.getCorrectedPosition());
        // Climber.fl.talon.setSensorPhase(false);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString code to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
         * switch(autoSelected) { case "My Auto": autonomousCommand = new
         * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
         * ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.start();
        }

        autoTimer.start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();

        // if (autoTimer.get() <= bookitTime) {
        //     driveTrainSubsystem.runDrive(1, -1);
        // } else if (autoTimer.get() > bookitTime && autoTimer.get() <= bookitTime + 0.25) {
        //     // DriveTrainSubsystem.leftFront.setIdleMode(IdleMode.kBrake);
        //     // DriveTrainSubsystem.leftRear.setIdleMode(IdleMode.kBrake);
        //     // DriveTrainSubsystem.rightFront.setIdleMode(IdleMode.kBrake);
        //     // DriveTrainSubsystem.rightRear.setIdleMode(IdleMode.kBrake);
        //     // driveTrainSubsystem.runDrive(0.75, -0.75);
        //     driveTrainSubsystem.runDrive(0.75, 0.75);
        // } else {
            rampingSubsystem.ramp(conveyorState);
            stick = -rampingSubsystem.wheel;
            wheel = rampingSubsystem.speed;
            leftSpeed = stick - wheel;
            rightSpeed = stick + wheel;
            driveTrainSubsystem.runDrive(leftSpeed, rightSpeed);
        //}
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
        Climber.leftPusher.set(ControlMode.PercentOutput, OI.xboxController.getRawAxis(3));
        Climber.rightPusher.set(ControlMode.PercentOutput, OI.xboxController.getRawAxis(3));
        // System.out.println(OI.operatorPanel.getRawAxis(3));

        if (OI.wheel.getRawButton(3)) {
            wheel = OI.xboxController.getRawAxis(1) * 0.75;
            stick = OI.wheel.getRawAxis(0);
        } else {
            rampingSubsystem.ramp(conveyorState);
            stick = -rampingSubsystem.wheel * 0.75;
            wheel = rampingSubsystem.speed;
        }

        // speedTurnScale = a * (1 / ((stick * stick) - h)) + k;
        speedTurnScale = 1;

        // wheel = OI.xboxController.getRawAxis(Constants.turnAxisPort);

        // rampingSubsystem.ramp(conveyorState);
        // stick = -rampingSubsystem.wheel;
        // wheel = rampingSubsystem.speed;


        // driveTrainSubsystem.runDrive(leftSpeed, rightSpeed);
        if (OI.operatorPanel.getRawButton(4)) {
            System.out.println("climbing, driver functionality disabled");
        } else {
            leftSpeed = stick - wheel;
            rightSpeed = stick + wheel;
            driveTrainSubsystem.runDrive(leftSpeed, rightSpeed);
            
        }
        // if(OI.xboxController.getRawButton(1)){
        // driveTrainSubsystem.leftRear.set(0.3);
        // driveTrainSubsystem.rightRear.set(0.3);
        // }

        // if(OI.xboxController.getRawButton(1)){
        // driveTrainSubsystem.leftRear.set(0.3);
        // driveTrainSubsystem.rightRear.set(0.3);
        // }

        if (OI.operatorPanel.getRawButton(14)) {
            conveyorSubsystem.tiltConveyor(ConveyorState.MID);
            conveyorState = ConveyorState.MID;
        }
        if (OI.operatorPanel.getRawButton(13)) {
            conveyorSubsystem.tiltConveyor(ConveyorState.LOW);
            conveyorState = ConveyorState.LOW;
        }
        if (OI.operatorPanel.getRawButton(15)) {
            conveyorSubsystem.tiltConveyor(ConveyorState.HIGH);
            conveyorState = ConveyorState.HIGH;
        }

        // System.out.println("wheel: " + wheel);
        System.out.println("Right side: " + pdp.getCurrent(0) + " " + pdp.getCurrent(1) + "    Left Side: " + pdp.getCurrent(15) + " " + pdp.getCurrent(14));
        System.out.println(DriveTrainSubsystem.leftFront.getOutputCurrent() + " " + DriveTrainSubsystem.leftRear.getOutputCurrent() + " " + DriveTrainSubsystem.rightFront.getOutputCurrent() + " " + DriveTrainSubsystem.rightRear.getOutputCurrent());

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        // System.out.println("Climber position:" +
        // this.testModule.getCorrectedPosition());
        climber.climberPeriodic();
        // System.out.println(Climber.fr.getName() + " is at
        // "+Climber.fr.getCorrectedPosition());
        // System.out.println(Climber.rl.getName() + " is outputting
        // "+Climber.fr.talon.getMotorOutputPercent());
        // System.out.printf("FL: %f, RL: %f, FR: %f, RR: %f",
        // DriveTrainSubsystem.leftFront.getOutputCurrent(),
        // DriveTrainSubsystem.leftRear.getOutputCurrent(),
        // DriveTrainSubsystem.rightFront.getOutputCurrent(),
        // DriveTrainSubsystem.rightRear.getOutputCurrent());
        // System.out.println(DriveTrainSubsystem.leftRear.getOutputCurrent());
        // System.out.println();
    }
}
