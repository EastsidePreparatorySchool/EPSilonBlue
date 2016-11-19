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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cliu on 11/11/2016.
 */
@Autonomous(name = "Beacon Selector", group = "Twitchy")
@Disabled
public class BeaconSelector extends LinearOpMode {
    /* Declare OpMode members. */

    String MY_COLOR = "RED";
    String OPPONENT_COLOR = "BLUE";
    int lightDetected = 2;
    double LEFT_POS = 0.5;
    double RIGHT_POS = 0.2;
    double MIDDLE_POS = 0.35;
    double CANNON_POS = 0.7;
    float RED_HUE = 15;   //10 - 15
    float BLUE_HUE = 270; //230 - 270


    HardwareTwitchy robot = new HardwareTwitchy();   // Use a Pushbot's hardware
    ColorSensor beaconSensor;
    //ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device


    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};




    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        beaconSensor = hardwareMap.colorSensor.get("beaconColor");

        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        double curPosition = robot.ballPicker.getPosition();

        //passive mode for beacon;
        beaconSensor.enableLed(false);
        robot.beaconSelector.setPosition(MIDDLE_POS);
        robot.ballPicker.setPosition(0.5);
        boolean flag = true;
        String beaconPos;
        waitForStart();


        while (opModeIsActive()) {

            while(flag == true){
                beaconPos = findLeftOrRight();

                    if (beaconPos.equals("LEFT")) {
                        robot.beaconSelector.setPosition(LEFT_POS);
                        telemetry.addData(">", "left pressed");
                        telemetry.update();
                        flag = false;
                        sleep(2000);
                    } else if (beaconPos.equals("RIGHT")) {
                        robot.beaconSelector.setPosition((RIGHT_POS));
                        telemetry.addData(">", "right pressed");
                        telemetry.update();
                        flag = false;
                        sleep(2000);
                    } else {
                        robot.beaconSelector.setPosition(MIDDLE_POS);
                        sleep(1000);
                    }
                findHue();

            }
            robot.beaconSelector.setPosition(CANNON_POS);
            telemetry.addData(">","action completed");

            telemetry.update();
            idle();
        }

    }



    //find the hue from bottomColor and print the color values to telemetry.
    public float findHue (){

        //return variable
        float hue = 0;
        beaconSensor.enableLed(false);
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        // convert the RGB values to HSV values.
        Color.RGBToHSV(beaconSensor.red() * 8, beaconSensor.green() * 8, beaconSensor.blue() * 8, hsvValues);

        //float hue is for return the value of hue. each color has a unique hue.
        hue = hsvValues[0];

        // send the info back to driver station using telemetry function.

        telemetry.addData("Clear", beaconSensor.alpha());
//        telemetry.addData("Red  ", beaconSensor.red());
//        telemetry.addData("Green", beaconSensor.green());
//        telemetry.addData("Blue ", beaconSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);


        telemetry.update();

        return hue;
    }


    public String findLeftOrRight () {

        String side = "";
        double BEACON_HUE;
        String BEACON_COLOR = "";
        //if detects light, then determine what color?

        while (beaconSensor.alpha() >= lightDetected) {
            BEACON_HUE = findHue();

            //determine the name of the color;
            if (BEACON_HUE >= 0 && BEACON_HUE <= 20) {
                BEACON_COLOR = "RED";
            } else if (230 <= BEACON_HUE && BEACON_HUE <= 270 ) {
                BEACON_COLOR = "BLUE";
            }


            if (BEACON_COLOR.equals(MY_COLOR)) {
                side = "LEFT";
            }else if (BEACON_COLOR.equals(OPPONENT_COLOR)){
                side = "RIGHT";
            }

            //telemetry.addData("alpha", beaconSensor.alpha());
            telemetry.addData(">", side);
            telemetry.update();

            return side;

        }
        return "";
    }
}


