/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "color sensor".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "AutoLineFollowing", group = "Twitchy")
//@Disabled
public class AutoLineFollowing extends LinearOpMode {

    //white hue is about 300
    int whiteHueValue = 300;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};


    ColorSensor bottomColor;    // Hardware Device Object
    HardwareTwitchy   robot         = new HardwareTwitchy();


    @Override
    public void runOpMode() throws InterruptedException {



        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.




        robot.init(hardwareMap);

        // get a reference to our ColorSensor object.
        bottomColor = hardwareMap.colorSensor.get("bottomColor");


        //set up the motors
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set the LED in the beginning to active mode
        bottomColor.enableLed(bLedOn);


        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data and determine motion

        while (opModeIsActive()) {

            findHue();

            //if hue is same as compare to whitehue,
            //then keep going straight,
            //if hue is not the same as whitehue,
            //wiggle around until sees the white hue, and stop
            //keep going straight
            //continue...


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }



    //find the hue from bottomColor and print the color values to phone.
    public float findHue (){

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

        telemetry.update();

        // change the background color to match the color detected by the RGB sensor.         // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });


        return hue;
    }




}
