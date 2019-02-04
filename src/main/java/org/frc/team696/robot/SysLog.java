/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc.team696.robot;

import java.util.TimerTask;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.RobotController;

public class SysLog extends TimerTask{
    public final static Integer slowIterations = 10;
    
    private Logger logger;
    private Integer iterCounter=0;

    public void init(){
        this.logger = LogManager.getLogger("Core");
    }

    public void run(){
        this.fastTasks();
        if(iterCounter == slowIterations){
            this.slowTasks();
            iterCounter = 0;
        }
        else{
            iterCounter++;
        }
    }

    private void slowTasks(){
        double voltage = RobotController.getBatteryVoltage();
        logger.info(String.format("Battery voltage: %.1f", voltage));
    }

    private void fastTasks(){
        
    }
}