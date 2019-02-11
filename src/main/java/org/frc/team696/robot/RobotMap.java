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
    public static final int flClimberTalon = 2;
    public static final boolean flClimberModuleInverted = true;
    public static final boolean flClimberModuleSensorPhase = true;
    
    //Front right
    public static final int frClimberTalon = 1;
    public static final boolean frClimberModuleInverted = true;
    public static final boolean frClimberModuleSensorPhase = false;

    //Rear left
    public static final int rlClimberTalon = 3;
    public static final boolean rlClimberModuleInverted = true;
    public static final boolean rlClimberModuleSensorPhase = false;

    //Rear right
    public static final int rrClimberTalon = 4;
    public static final boolean rrClimberModuleInverted = true;
    public static final boolean rrClimberModuleSensorPhase = false;

    
}
