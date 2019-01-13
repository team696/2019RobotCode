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

    /*
        RGB Address
     */

    public static byte deviceAddress = 0x29;
    public static byte enableAddress = 0x00;
    public static byte commandBit = (byte) 0x80;
    public static byte enable_AEN = 0x02;
    public static byte enable_PON = 0x01;

    // Channel Data

    public static byte clearDataL = 0x14;
    public static byte clearDataH = 0x15;
    public static byte redDataL = 0x16;
    public static byte redDataH = 0x17;
    public static byte greenDataL = 0x18;
    public static byte greenDataH = 0x19;
    public static byte blueDataL = 0x1A;
    public static byte blueDataH = 0x1B;

    // Drive Train Talon IDs

    public static int lFront = 14;
    public static int lMid = 15;
    public static int lRear = 16;
    public static int rFront = 1;
    public static int rMid = 2;
    public static int rRear = 3;

    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
}
