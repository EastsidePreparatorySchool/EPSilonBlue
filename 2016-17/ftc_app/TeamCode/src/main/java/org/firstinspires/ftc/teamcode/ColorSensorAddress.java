package org.firstinspires.ftc.teamcode;

/**
 * Created by cliu on 11/17/2016.
 */

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cliu on 11/11/2016.
 */
@Autonomous(name = "Beacon Selector LA", group = "Twitchy")
@Disabled
public class ColorSensorAddress extends LinearOpMode {
    /* Declare OpMode members. */

    String MY_COLOR = "RED";
    String OPPONENT_COLOR = "BLUE";
    int lightDetected = 2;


    HardwareTwitchy robot = new HardwareTwitchy();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    ColorSensor beaconSensor;
    ColorSensor bottomSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        beaconSensor = hardwareMap.colorSensor.get("beaconColor");
        bottomSensor = hardwareMap.colorSensor.get("bottomColor");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        bottomSensor.setI2cAddress(I2cAddr.create7bit(0x4c >> 1));



    }

}
