/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    //Climber parameters
    //Front left
    public static final int flClimberTalon = 1;
    public static final boolean flClimberModuleInverted = true;
    public static final boolean flClimberModuleSensorPhase = true;
    
    //Front right
    public static final int frClimberTalon = 14;
    public static final boolean frClimberModuleInverted = false;
    public static final boolean frClimberModuleSensorPhase = true;

    //Rear left
    public static final int rlClimberTalon = 7;
    public static final boolean rlClimberModuleInverted = false;
    public static final boolean rlClimberModuleSensorPhase = true;

    //Rear right
    public static final int rrClimberTalon = 8;
    public static final boolean rrClimberModuleInverted = true;
    public static final boolean rrClimberModuleSensorPhase = true;

    //Left pusher
    public static final int leftPusherTalon = 6;
    public static final boolean leftPusherInvert = false;

    //Right pusher
    public static final int rightPusherTalon = 9;
    public static final boolean rightPusherInvert = true;
    
    //Conveyor parameters
    public static int topConveyorMotorPort=5;
    public static int bottomConveyorMotorPort=10;
    public static int conveyorSolPortTop = 0;
    public static int conveyorSolPortBottom = 1;
    
    //Drivetrain parameters
    public static int leftFrontPort = 2;
    public static int leftMidPort = 3;
    public static int leftRearPort = 4;

    public static int rightRearPort = 11;
    public static int rightMidPort = 12;
    public static int rightFrontPort = 13;


    
}
