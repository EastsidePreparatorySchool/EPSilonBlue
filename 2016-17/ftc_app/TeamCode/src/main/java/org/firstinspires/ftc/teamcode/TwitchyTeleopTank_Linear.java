/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Timer;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 * <p>
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Twitchy: Telop Tank", group = "Twitchy")

public class TwitchyTeleopTank_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Timer timer = new Timer();
    double x;
    double y;
    double lPower;
    double rPower;

    double max;
    double h;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor raiser;
    DcMotor cannon;
    Servo beacon;
    Servo picker;
    double pickPosition;

    @Override
    public void runOpMode() throws InterruptedException {


        // link motors names with actual motors
        leftMotor = hardwareMap.dcMotor.get("motorLeft");
        rightMotor = hardwareMap.dcMotor.get("motorRight");
        raiser = hardwareMap.dcMotor.get("raiser");
        cannon = hardwareMap.dcMotor.get("cannon");
        beacon = hardwareMap.servo.get("pusher");
        picker = hardwareMap.servo.get("picker");


        //set the two backward motors to run in reverse
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        raiser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cannon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;

            //Todo: che Math
//            h = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
//            lPower = (x+h)/2;
//            rPower = lPower-x;
//
//
//            //exception if just x for turning
//            if(y<0.1 && y>-0.1 && x>0.1){
//                leftMotor.setPower(1.0);
//                rightMotor.setPower(-1.0);
//            } else if(y<0.1 && y>-0.1 && x<0.1){
//                leftMotor.setPower(-1.0);
//                rightMotor.setPower(1.0);
//            }
//
//            lPower = Range.clip(lPower, -1, 1);
//            rPower = Range.clip(rPower, -1, 1);
//
//            leftMotor.setPower(lPower);
//            rightMotor.setPower(rPower);


//            TODO: quinn math
//            lPower = Math.pow((Math.pow(y,2)+ Math.pow(x,2)-(2*y*x))/2,1/2);
//            rPower = Math.pow((Math.pow(y,2)+ Math.pow(x,2)+(2*y*x))/2,1/2);
//            if(-x>y){
//                rPower = -1 *rPower;
//            }
//
//            if(x>y){
//                lPower = -1*lPower;
//            }
//
//            lPower = Range.clip(lPower, -1, 1);
//            rPower = Range.clip(rPower, -1, 1);
//
//            leftMotor.setPower(lPower);
//            rightMotor.setPower(rPower);

            // TODO: POV

            lPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x) * 0.6;
            rPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x) * 0.6;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(lPower), Math.abs(rPower));
            if (max > 1.0) {
                lPower /= max;
                rPower /= max;
            }
            if (gamepad1.left_trigger > 0.25) {
                lPower = lPower / 2;
                rPower = rPower / 2;
            } if (gamepad1.start){
                lPower = lPower * (4/3);
                rPower = rPower * (4/3);
            }

            leftMotor.setPower(lPower);
            rightMotor.setPower(rPower);

            //

//            TODO:tank drive
//            lPower = -gamepad1.left_stick_y;
//            rPower = -gamepad1.right_stick_y ;
//            leftMotor.setPower(lPower);
//            rightMotor.setPower(rPower);
//

            // right trigger is fire cannon
            if (gamepad1.right_trigger > 0.25) {
                cannon.setPower(-1.0);
            } else {
                cannon.setPower(0.0);
            }

            // cannon raise by right joystick
            raiser.setPower(gamepad1.right_stick_y / 3);

            // use dpad to shift between three positions
            if (gamepad1.dpad_left) {
                beacon.setPosition(0.2);
            } else if (gamepad1.dpad_up) {
                beacon.setPosition(0.7);
            } else if (gamepad1.dpad_down) {
                beacon.setPosition(0.4);
            } else if (gamepad1.dpad_right) {
                beacon.setPosition(0.55);
            }


            // Use y and a to raise and lower the picker
            if (gamepad1.y)
                pickPosition = 0.2;
            else if (gamepad1.a)
                pickPosition = 0.75;

            // Move servo to new position.
            pickPosition = Range.clip(pickPosition, 0.0, 0.7);
            picker.setPosition(pickPosition);

//
//            // Send telemetry message to signify robot running;
//            telemetry.addData("arm",   "%.2f", armPosition);
//            telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("My name is Hal_9000", " ");
            telemetry.addData("left power", "%.2f", lPower);
            telemetry.addData("right power", "%.2f", rPower);
            telemetry.update();


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        }
    }
}

//.5 seconds half power