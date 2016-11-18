package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cliu on 11/11/2016.
 */
@Autonomous(name = "twoColorSensors", group = "Twitchy")
//@Disabled
public class twoColorSensors extends LinearOpMode {
    /* Declare OpMode members. */

    HardwareTwitchy robot   = new HardwareTwitchy();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device
    ColorSensor bottomColor;
    ColorSensor beaconColor;


    //creates variables for color sensors
    // bLedOn represents the state of the LED.
    boolean bLedOn = true;
    boolean bLedOff = false;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};




    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");   //get reference for gyro
        bottomColor = hardwareMap.colorSensor.get("bottomColor");     // get a reference to our ColorSensor object.
        beaconColor = hardwareMap.colorSensor.get("beaconColor");
        beaconColor.setI2cAddress(I2cAddr.create7bit(0x5c >> 1));
        bottomColor.setI2cAddress(I2cAddr.create7bit(0x3c >> 1));

        bottomColor.enableLed(true);
        beaconColor.enableLed(false);
        sleep(2000);

        waitForStart();
        while (opModeIsActive()){
            findBottomHue();

            idle();
        }



    }


    //find the hue from bottomColor and print the color values to telemetry.
    public float findBottomHue (){



        //return variable
        float hue = 0;

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        // convert the RGB values to HSV values.
        Color.RGBToHSV(bottomColor.red() * 8, bottomColor.green() * 8, bottomColor.blue() * 8, hsvValues);

        //float hue is for return the value of hue. each color has a unique hue.
        hue = hsvValues[0];

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", bottomColor.alpha());
        telemetry.addData("Red  ", bottomColor.red());
        telemetry.addData("Green", bottomColor.green());
        telemetry.addData("Blue ", bottomColor.blue());
        telemetry.addData("Hue", hsvValues[0]);

        telemetry.addData("beacon address", beaconColor.getI2cAddress());
        telemetry.addData("bottom address", bottomColor.getI2cAddress());

        telemetry.update();



        return hue;
    }



}
