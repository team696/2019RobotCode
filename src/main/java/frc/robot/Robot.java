/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot  {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // private static final String k_path_name = "boz";

  // public static CANSparkMax leftFront;
  // public static CANSparkMax leftRear;

  // public static CANSparkMax rightFront;
  // public static CANSparkMax rightRear;
  

  // public static CANEncoder lFrontEncoder;
  // public static CANEncoder lRearEncoder;

  // public static CANEncoder rFrontEncoder;
  // public static CANEncoder rRearEncoder;

  // public static SpeedControllerGroup leftSide;
  // public static SpeedControllerGroup rightSide;

  // public static DifferentialDrive drive;  


  public static AHRS navX = new AHRS(Port.kMXP, (byte)50);
  Compressor comp = new Compressor(17);

  Trajectory leftTrajectory;
  Trajectory rightTrajectory;


  public static EncoderFollower leftFollower;
  public static EncoderFollower rightFollower;

  public static Notifier followerNotifier;

  // public static double conversionFactor = 0.16;

  CANSparkMax leftFront = new CANSparkMax(2, MotorType.kBrushless);
  CANEncoder lFrontEncoder = new CANEncoder(leftFront);
  // lFrontEncoder.setPositionConversionFactor(1.9188888888888888888);

  CANSparkMax leftRear = new CANSparkMax(4, MotorType.kBrushless);
  CANEncoder lRearEncoder = new CANEncoder(leftRear);

  CANSparkMax rightFront = new CANSparkMax(13, MotorType.kBrushless);
  CANEncoder rFrontEncoder = new CANEncoder(rightFront);
  // rFrontEncoder.setPositionConversionFactor(1.9188888888888888888);

  CANSparkMax rightRear = new CANSparkMax(11, MotorType.kBrushless);
  CANEncoder rRearEncoder = new CANEncoder(rightRear);

  SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftRear);
  SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightRear);

  PrintWriter errorFileWriter;

  double wheelDiameter =  5.5/12;

  public static String path1 = "Unnamed";
  public static String path2 = "path2";


  VictorSPX conveyor = new VictorSPX(10);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());

    //just added 
    navX.zeroYaw();
    lFrontEncoder.setPosition(0);
    rFrontEncoder.setPosition(0);

    lFrontEncoder.setPosition(0);
    lRearEncoder.setPosition(0);
    rFrontEncoder.setPosition(0);
    rRearEncoder.setPosition(0);

    
    leftFront.restoreFactoryDefaults();
    leftFront.setSmartCurrentLimit(80, 20, 10000);
    leftFront.setOpenLoopRampRate(0);
    //lFrontEncoder.setPositionConversionFactor(0.1538);
    
    leftRear.restoreFactoryDefaults();
    leftRear.setSmartCurrentLimit(80, 20, 10000);
    leftRear.setOpenLoopRampRate(0);
    // lRearEncoder.setPositionConversionFactor(conversionFactor);

    rightFront.restoreFactoryDefaults();
    rightFront.setSmartCurrentLimit(80, 20, 10000);
    rightFront.setInverted(true);
    rightFront.setOpenLoopRampRate(0);
    //rFrontEncoder.setPositionConversionFactor(0.1463);


    rightRear.restoreFactoryDefaults();
    rightRear.setSmartCurrentLimit(80, 20, 10000);
    rightRear.setInverted(true);
    rightRear.setOpenLoopRampRate(0);
    // rRearEncoder.setPositionConversionFactor(conversionFactor);
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
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
    m_autonomousCommand = m_chooser.getSelected();

    //File logFile = new File("C:/Users/atar1/Desktop/csv/logFile.csv") ;
   

    // navX.zeroYaw();

    int leftIntPos = (int)(lFrontEncoder.getPosition());
    int rightIntPos = (int)(rFrontEncoder.getPosition());

 try{


    Trajectory left_trajectory =  PathfinderFRC.getTrajectory(k_path_name + ".right");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
  
    leftFollower = new EncoderFollower(left_trajectory);
    rightFollower = new EncoderFollower(right_trajectory);


    leftFollower.configureEncoder(leftIntPos, 10, wheelDiameter);
    leftFollower.configurePIDVA(0.5,0,0,1/16, 0);

    rightFollower.configureEncoder(rightIntPos , 10 , wheelDiameter);
    rightFollower.configurePIDVA(0.5, 0, 0, 1/16, 0);

    followerNotifier1 = new Notifier(this::followPath1);
    followerNotifier1.startPeriodic(leftTrajectory1.get(0).dt);
    
   






  }
  catch(IOException ex){
    System.out.println("Error ignored");
    System.out.println(ex);
  }










    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }



  
  public void followPath(){
    if(leftFollower.isFinished()||rightFollower.isFinished()){
      followerNotifier.stop();
      leftSide.set(0);
      rightSide.set(0);
      System.out.println("pathfinder is done");

    } else {
      leftFront.setIdleMode(IdleMode.kCoast);
      leftRear.setIdleMode(IdleMode.kCoast);
      rightFront.setIdleMode(IdleMode.kCoast);
      rightRear.setIdleMode(IdleMode.kCoast);

      double rawLeftSpeed = leftFollower.calculate((int)lFrontEncoder.getPosition());
      double rawRightSpeed = rightFollower.calculate((int)rFrontEncoder.getPosition());
      double heading = navX.getYaw();
      double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(heading - desired_heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      double leftSpeed = rawLeftSpeed+turn;
      double rightSpeed = rawRightSpeed-turn;
      // drive.tankDrive(leftSpeed, rightSpeed);
      leftSide.set(leftSpeed);
      rightSide.set(rightSpeed);
      System.out.println(rawRightSpeed);

      //csv file writer

      // try{
      //   //  FileWriter logFile = new FileWriter(new FileWriter("log.csv"));
      //    errorFileWriter = new PrintWriter(new File("log.csv"));
      //    StringBuffer csvHeader = new StringBuffer("");
      //    StringBuffer csvData = new StringBuffer("");


      //    csvHeader.append("dTime,SegPosition,DistanceCovered\n");

      //    errorFileWriter.write(csvHeader.toString());
        
      //   for(int i = 0; i<200; i++){
      
      //     csvData.append((char)leftFollower.getSegment().dt);
      //     csvData.append(',');
      //     csvData.append((char)leftFollower.getSegment().position);
      //     csvData.append(',');
      //     csvData.append((char)leftFollower.getSegment().x);
      //     csvData.append('\n');
      //     errorFileWriter.write(csvData.toString());


      //   } 
      //   errorFileWriter.close();
        
      // }
      //    catch(FileNotFoundException error){

      //      error.printStackTrace();
      //    }
     
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //System.out.println(lFrontEncoder.getPosition());
    
  } 

  @Override
  public void teleopInit() {
    
    followerNotifier.stop();
    leftSide.set(0);
    rightSide.set(0); 
    navX.zeroYaw();
    lFrontEncoder.setPosition(0);
    rFrontEncoder.setPosition(0);
    System.out.println("pathfinder is over");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    comp.start();
    
    // System.out.println("navX   lFront   lRear   rFront   rRear");
    // System.out.println(navX.getYaw()+"   "+lFrontEncoder.getPosition()+"   "+lRearEncoder.getPosition()+"   "+rFrontEncoder.getPosition()+"   "+rRearEncoder.getPosition());
    // System.out.println(navX.getYaw());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    comp.start();
  }
}
