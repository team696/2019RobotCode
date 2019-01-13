/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeMotorCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;

  public  double elevatorUpSpeed = 0.5; 
  public  double elevatorDownSpeed = -0.5;

  public double intakeSpeedIn = 0.5;
  public double intakeSpeedOut = -0.5;

  public boolean isBrake = true;
  public boolean intakeIsBrake = true;
  public static boolean currentIntakeState;
  public boolean oldIntakeState;
  public boolean toggleIntakeState;

  public boolean oldTiltState;
  public boolean toggleTiltState;
  public boolean currentTiltState;

  public double leftSpeed;
  public double rightSpeed;

  public double speed;
  public double turn;

  public double forwardSpeed;
  public double backwardSpeed;

  public double speedMultiplier = 0.85;
  public double turnMultiplier = 0.65;

  public int loopNumber;

public static DriveSubsystem driveSubsystem = new DriveSubsystem(RobotMap.lFrontPort, RobotMap.lMiddlePort, RobotMap.lRearPort, RobotMap.rFrontPort, RobotMap.rMiddlePort, RobotMap.rRearPort);
public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(RobotMap.elevatorPortNumber);
public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.intakePortNumberA, RobotMap.intakePortNumberB);
public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem(RobotMap.elevatorBrakePortNumber,RobotMap.tiltPortNumber,RobotMap.intakeSolPortNumber);
  
public static Compressor compressor = new Compressor();

  // Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Command autonomousCommand;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    // chooser.addObject("My Auto", new MyAutoCommand());

    SmartDashboard.putData("Auto mode", m_chooser);
    CameraServer.getInstance().addAxisCamera("10.6.96.66");


    // m_chooser.addObject("Intake", new  IntakeSolCommand()) ;

    elevatorSubsystem.zeroElevator();


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

    autonomousCommand = m_chooser.getSelected();
    


    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
  
    if(autonomousCommand != null){
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

    compressor.start();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    
    Scheduler.getInstance().run();


    // Elevator if Statements
    //  if (OI.xboxController.getRawButton(Constants.elevatorUpButton))
    // {
    //   pneumaticsSubsystem.runDiscBrake(true);
    //   elevatorSubsystem.runElevatorUp(elevatorUpSpeed);
    // }
    // else if (OI.xboxController.getRawButton(Constants.elevatorDownButton))
    // {
    //   pneumaticsSubsystem.runDiscBrake(true);
    //   elevatorSubsystem.runElevatorDown(elevatorDownSpeed);
    // }
   
    // else{
    //   elevatorSubsystem.stopElevator();
    //   pneumaticsSubsystem.runDiscBrake(false);
    // }



// OI.intakeButton.whileActive(new IntakeCommand(true));
// OI.intakeButton.whenInactive(new IntakeCommand(false));

OI.intakeButton.whenPressed(new IntakeCommand(true));
//OI.intakeButton.toggleWhenActive(new IntakeCommand(true));

// Intake if Statements
  currentIntakeState = OI.xboxController.getRawButton(Constants.intakeButtonIn);
  if(currentIntakeState && !oldIntakeState){
    toggleIntakeState = !toggleIntakeState;
  }
  oldIntakeState = currentIntakeState;
  if (toggleIntakeState){
    pneumaticsSubsystem.runIntakeBrake(true);
    intakeSubsystem.runIntake(intakeSpeedIn);
  }  
// if(OI.xboxController.getRawButton(Constants.intakeButtonIn)){
//       pneumaticsSubsystem.runIntakeBrake(true);
//       intakeSubsystem.runIntake(intakeSpeedIn);
//     }
    else {
      intakeSubsystem.runIntake(0);
      pneumaticsSubsystem.runIntakeBrake(false);
    }
  
    
//tilt if statements

//new toggle ifs
// currentTiltState = OI.xboxController.getRawButton(2);

// if(currentTiltState && !oldTiltState ){
//   toggleTiltState=!toggleTiltState;
// }

// oldTiltState = currentTiltState;


// //System.out.println(toggleTiltState);
// if(toggleTiltState){
//   pneumaticsSubsystem.runTilt(true);
// } else{
//   pneumaticsSubsystem.runTilt(false);
// }  

//gta v controls
double forwardSpeed = speedMultiplier*OI.xboxController.getRawAxis(3);
double backwardSpeed = speedMultiplier*OI.xboxController.getRawAxis(2);
double turn = turnMultiplier*OI.xboxController.getRawAxis(0);

speed = forwardSpeed - backwardSpeed;


   leftSpeed = speed+turn;
   rightSpeed = speed-turn;


  // Drive if Statements
   driveSubsystem.runDrive(leftSpeed,rightSpeed);
  
 
}
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
