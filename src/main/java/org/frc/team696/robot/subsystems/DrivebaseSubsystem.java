/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class DrivebaseSubsystem extends Subsystem {
    private WPI_TalonSRX leftFrontTalon;
    private WPI_TalonSRX leftMidTalon;
    private WPI_TalonSRX leftRearTalon;

    private WPI_TalonSRX rightFrontTalon;
    private WPI_TalonSRX rightMidTalon;
    private WPI_TalonSRX rightRearTalon;

    private DifferentialDrive drive;

    public DrivebaseSubsystem(int leftFrontAddress, int leftMidAddress, int leftRearAddress,
                              int rightFrontAddress, int rightMidAddress, int rightRearAddress){
        this.leftRearTalon = new WPI_TalonSRX(leftRearAddress);
        this.leftMidTalon = new WPI_TalonSRX(leftMidAddress);
        this.leftFrontTalon = new WPI_TalonSRX(leftFrontAddress);

        this.rightRearTalon = new WPI_TalonSRX(rightRearAddress);
        this.rightMidTalon = new WPI_TalonSRX(rightMidAddress);
        this.rightFrontTalon = new WPI_TalonSRX(rightFrontAddress);

        this.leftFrontTalon.set(ControlMode.PercentOutput, 0);
        this.leftRearTalon.set(ControlMode.Follower, leftFrontAddress);
        this.leftMidTalon.set(ControlMode.Follower, leftFrontAddress);

        this.rightFrontTalon.set(ControlMode.PercentOutput, 0);
        this.rightRearTalon.set(ControlMode.Follower, rightFrontAddress);
        this.rightMidTalon.set(ControlMode.Follower, rightFrontAddress);

        this.drive = new DifferentialDrive(this.leftFrontTalon, this.rightFrontTalon);

    }

    public void tankDrive(double leftPower, double rightPower) {
        drive.tankDrive(leftPower, rightPower);
    }

    public void arcadeDrive(double speed, double rotation){
        drive.arcadeDrive(speed, rotation);
    }

    public void initDefaultCommand() {
        // TODO: Set the default command for a subsystem here. Example:
        //    setDefaultCommand(new MySpecialCommand());
    }
}
