package org.frc.team696.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrainSubsystem extends Subsystem {

    public  WPI_TalonSRX leftRear;
    public  WPI_TalonSRX leftMid;
    public  WPI_TalonSRX leftFront;

    public  WPI_TalonSRX rightRear;
    public  WPI_TalonSRX rightMid;
    public  WPI_TalonSRX rightFront;

    public SpeedControllerGroup leftSide;
    public SpeedControllerGroup rightSide;

    private DifferentialDrive drive;


    public DriveTrainSubsystem(int leftRear, int leftMid, int leftFront,
                               int rightRear, int rightMid, int rightFront) {

        /*
            Initializing TalonSRX Objects and PID Controller
         */

        this.leftRear = new WPI_TalonSRX(leftRear);
        this.leftMid = new WPI_TalonSRX(leftMid);
        this.leftFront = new WPI_TalonSRX(leftFront);

        this.rightRear = new WPI_TalonSRX(rightRear);
        this.rightMid = new WPI_TalonSRX(rightMid);
        this.rightFront = new WPI_TalonSRX(rightFront);


        leftSide = new SpeedControllerGroup(this.leftRear, this.leftMid, this.leftFront);
        rightSide = new SpeedControllerGroup(this.rightRear, this.rightMid, this.rightFront);

        this.drive = new DifferentialDrive(this.leftSide, this.rightSide);



    }

    public void tankDrive(double leftDrive, double rightDrive) {

        drive.tankDrive(leftDrive, rightDrive);

    }

    @Override
    public void initDefaultCommand() {

    }

}
