/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import org.frc.team696.robot.autonomousCommands.Default;
import org.frc.team696.robot.commands.DriveBackCommand;
import org.frc.team696.robot.commands.DriveToAngleCommand;
import org.frc.team696.robot.subsystems.DriveToAngle;
import org.frc.team696.robot.subsystems.DriveTrainSubsystem;

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

    public static final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.lRear, RobotMap.lMid, RobotMap.lFront, 
                                                                                          RobotMap.rRear, RobotMap.rMid, RobotMap.rFront);
    
    public static DriveToAngle driveToAngleSubsystem;                                                                                  

    private Command autonomousCommand;
    private SendableChooser<Command> chooser = new SendableChooser<>();


    public static DigitalInput leftIRSensor = new DigitalInput(0);
    public static DigitalInput rightIRSensor = new DigitalInput(1);

    public static DriveBackCommand command = new DriveBackCommand(45);

    //public static IMUProtocol navX;

    public static AHRS navX;
    SerialPort port;
    byte UpdateRateHz;


    double speed;
    double turn;
    double leftValue;
    double rightValue;

    boolean gotLeft = false;
    boolean gotRight = false;
    boolean isReady = false;

    double x =259.8; //2.273*218;
    double y; 
    
    double targetAngle;
    static double targetAngleDegrees;
    public static double finalTargetAngle;

   public static double initialEncoder;
   public static double leftEncoder;
   public static double rightEncoder;
   public static double finalEncoder;
   public static double encoderDifference;
   public boolean angleGot = false;
   public DriveToAngleCommand driveToAngleCommand = new DriveToAngleCommand(45);
   Command driveToAngle;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {

        driveTrainSubsystem.leftRear.setSelectedSensorPosition(0);
        driveTrainSubsystem.rightFront.setSelectedSensorPosition(0);

        
        oi = new OI();
        chooser.setDefaultOption("Default Auto", new Default());

        // chooser.addObject("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", chooser);

        try {
            UpdateRateHz = 50;
            port = new SerialPort(57600, SerialPort.Port.kMXP);
            // navX = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData,UpdateRateHz); 
            navX = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, UpdateRateHz);

        } catch(Exception ex){System.out.println("NavX not working");}

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

        
        // System.out.println("Left:   " + leftIRSensor.get() + "     " + "Right:  " + !rightIRSensor.get());
        
        if (leftIRSensor.get() && !gotLeft){
        gotLeft = true;
        leftEncoder = (driveTrainSubsystem.rightFront.getSelectedSensorPosition()+driveTrainSubsystem.leftRear.getSelectedSensorPosition()/2);  
        }

        if (!rightIRSensor.get() && !gotRight){
            gotRight = true;
            rightEncoder = (driveTrainSubsystem.rightFront.getSelectedSensorPosition()+driveTrainSubsystem.leftRear.getSelectedSensorPosition()/2);  
           // navX.zeroYaw();
            }
    
        if(gotLeft && gotRight){
            // finalEncoder = (driveTrainSubsystem.rightFront.getSelectedSensorPosition()+driveTrainSubsystem.leftRear.getSelectedSensorPosition())/2;
            // encoderDifference=finalEncoder-initialEncoder;
            encoderDifference = rightEncoder-leftEncoder;
            isReady = true;
            //navX.zeroYaw();

        }
        y = encoderDifference;
        // System.out.println("y"+"    "+y);
      //  System.out.println("Average Encoder:    " + (driveTrainSubsystem.rightFront.getSelectedSensorPosition()+driveTrainSubsystem.leftRear.getSelectedSensorPosition() / 2));
        //System.out.println(leftEncoder + "            " + rightEncoder);

        // OI.alignButton.whileHeld(new DriveCommand(0,targetAngle));


       


        speed = -OI.stick.getRawAxis(1) * 0.75;
        turn = -OI.stick.getRawAxis(4) * 0.5;


        if(turn < 0.1 && turn > -0.1){
            turn = 0;
        }

        leftValue = speed - turn;
        rightValue = speed + turn;


     

       targetAngle = Math.atan(x/y);

    //    targetAngleDegrees = Math.toDegrees(targetAngle);
    targetAngleDegrees = 45;


    //    if(!angleGot && gotLeft && gotRight ){
    // //    driveToAngleSubsystem = new DriveToAngle(targetAngle);
    //    System.out.println("Hello");
    //       angleGot = true;
    //    }

       driveToAngleSubsystem = new DriveToAngle(targetAngleDegrees);


       //driveToAngleCommand = new DriveToAngleCommand(45);

    //    if (OI.alignButton.wh{
    //         driveToAngleSubsystem.doAll();
            
    //     // driveToAngleCommand.start();
    //    }else{
    //         driveToAngleSubsystem.cancel();
    //         // double manualSpeed = OI.stick.getRawAxis(1);
    //         // double manualWheel = OI.stick.getRawAxis(4);
    //         // double leftDrive = manualSpeed + manualWheel;
    //         // double rightDrive = manualSpeed - manualWheel;
    //         // driveTrainSubsystem.tankDrive(-leftDrive, rightDrive);
    //         System.out.println("Target Angle:   "  + targetAngleDegrees);
    //    }
    // OI.alignButton.whileHeld(new DriveBackCommand(45));
    // System.out.println("Target Angle:   "  + targetAngleDegrees);
    if(OI.stick.getRawButton(1)){
        command.start();
    }



    //    System.out.println(leftEncoder+ "       " + rightEncoder);
        
      // System.out.println(driveTrainSubsystem.rightFront.getSelectedSensorPosition());

    //    driveToAngle = new DriveCommand(0, targetAngleDegrees); 

            

    // if(OI.stick.getRawButton(1)){
    //     driveToAngleSubsystem.getTargetAngle(targetAngleDegrees);   
    // }
    // else{
    //     driveToAngleSubsystem.cancel();
    // }


    // if(OI.stick.getRawButton(1)){
    //     DriveToAngle.start();
    // }
    // else {
    //     driveToAngle.cancel();
    // }
    // System.out.println(DriveToAngle.isRunning());
    // if(OI.stick.getRawButton(2)){
    //     gotLeft=false;
    //     gotRight=false;
    // }
    //    OI.alignButton.whenPressed(driveToAngle);

    //    if(isReady){
    //        driveToAngle.start();
    //    }
      







        // System.out.println("NAVX        " + navX.getYaw());
        // System.out.println("TURN   " + turn);
        // System.out.println(leftValue + "   " + rightValue);

        // System.out.println(Math.tan(45));

    }  

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
    }
}
