package org.frc.team696.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class RGBSensorSubsystem extends Subsystem {


    public I2C rgbSensor;
    private Timer time = new Timer();

    /*
        Buffer
     */

    private byte[] data = new byte[8];

    /*
        Channels Declaration
     */

    private int cData;
    private int red;
    private int green;
    private int blue;

    private double luminance;

    private double rgbAverage;
    private double redNoLuminance;
    private double greenNoLuminance;
    private double blueNoLuminance;
    private double difference;

    /*
        Addresses
     */

    private byte commandBit = (byte) 0x80;
    private byte sensorWaitTime = 0x03;
    private byte aTimeAddress = (byte) 0x81;

    /*
        Integration Times for RGB Sensor
     */

    private byte integrationTime_2_4ms = (byte) 0xFF;
    private byte integrationTime_14ms = (byte) 256 - 6;
    private byte integrationTime_24ms = (byte) 0xF6;
    private byte integrationTime_50ms = (byte) 0xEB;
    private byte integrationTime_101ms = (byte) 0xD5;
    private byte integrationTime_154ms = (byte) 0xC0;
    private byte integrationTime_700ms = (byte) 0x00;

    /*
        Gain Addresses
     */

    private byte gain_x1 = (byte) 0x00;
    private byte gain_x4 = (byte) 0x01;
    private byte gain_x16 = (byte) 0x02;
    private byte gain_x60 = (byte) 0x03;

    /*
        Instance Variables
    */

    private double whiteThreshold = 10;


    /**
     * Constructor for RGBSensorSubsystem class
     * @param deviceAddress the device address for the RGB sensor, which allows the robot to communicate with it
     */

    public RGBSensorSubsystem(byte deviceAddress) {

        this.rgbSensor = new I2C(I2C.Port.kOnboard, deviceAddress);

    }

    /**
     * Getting the values for red, blue, green, and cData
     */

    public void rgbGetLux() {

        // Select enable register
        // Power ON, RGBC enable, wait time disable
        rgbSensor.write(commandBit, sensorWaitTime);
        // Select ALS time register
        // Atime = 700 ms
        rgbSensor.write(aTimeAddress, integrationTime_14ms);
        // Select Wait Time register
        // WTIME : 50ms
        rgbSensor.write(0x83, integrationTime_14ms);
        // Select control register
        // AGAIN = 1x
        rgbSensor.write(0x8F, gain_x1);

        // Thread.sleep here to account for wait time register

        try {
            Thread.sleep(14);
        } catch (InterruptedException e) {
            System.out.println("If you get this message, then Ismail sucks");
        }

        /*
            Read 8 Bytes of Data
         */

        rgbSensor.read(0x94, 8, data);

        /*
            Conversion of Data Read
         */

        cData = ((data[1] & 0xFF) * 256) + (data[0] & 0xFF);
        red = ((data[3] & 0xFF) * 256) + (data[2] & 0xFF);
        green = ((data[5] & 0xFF) * 256) + (data[4] & 0xFF);
        blue = ((data[7] & 0xFF) * 256) + (data[6] & 0xFF);

        /*
            Luminance Final Calculation
         */

        luminance = (-0.32466 * red) + (1.57837 * green) + (-0.73191 * blue);


        /*
            No Luminance Calculation
        */

        rgbAverage = ((red + blue + green) / 3);
        redNoLuminance = (red / luminance);
        blueNoLuminance = (blue / luminance);
        greenNoLuminance = (green / luminance);

    }

    /**
     * Accessor method for redNoLuminance
     * @return the redNoLuminance value
     */

    public double getRed(){
        return redNoLuminance;
    }

    /**
     * Accessor method for blueNoLuminance
     * @return the blueNoLuminance value
     */

    public double getBlue(){
        return blueNoLuminance;
    }

    /**
     * Accessor method for greenNoLuminance
     * @return the greenNoLuminance value
     */

    public double getGreen() {
        return greenNoLuminance;
    }
    
    /**
     * Accessor method for gbAverage
     * @return the gbAverage value, giving us the white value of the RGB sensor 
     */

    public double getWhite() {
        return rgbAverage;
    }

    /**
     * Algorithm to detect whether ot not the RGB sensor is over the white line or not.
     * @return true if sensor is over white line, false if not
     */

    public boolean onWhiteLine() {
        if(rgbAverage > whiteThreshold){
            return true;
        }
        return false;
    }


    public void initDefaultCommand() {

    }
}
