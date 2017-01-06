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

@TeleOp(name = "Twitchy: Telop POV", group = "Twitchy")

public class TwitchyTeleopTank_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Timer timer = new Timer();
    double x;
    double y;
    double lPower;
    double rPower;

    double max;
    double h;


    double pickPosition;
    double beaconPosition;
    HardwareTwitchy robot = new HardwareTwitchy();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() throws InterruptedException {


        // link motors names with actual motors
        robot.init(hardwareMap);


        //set the two backward motors to run in reverse
        robot.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.raiser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.cannon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;


//            TODO: Control mapping
/*
            left joystick controls movement
            left bumper is slow mode
            right bumper is fast mode
            right trigger is fire the cannon
            cannon raising is right joystick
            y is raise picker
            a is lower picker
            dpad left is fire actuator
            dpad right is retract actuator
            free buttons: dpad up, dpad down, x, b,back and start and left trigger
 */
            // TODO: POV

            if(gamepad1.left_bumper){
                y*= robot.slowSpeed;
                x/= robot.slowTurn;
            } else if(gamepad1.right_bumper){
                y*= robot.fastSpeed;
                x/= robot.normalTurn;
            }else{
                y*= robot.normalSpeed;
                x/= robot.normalTurn;
            }



            lPower = y - x;
            rPower = y + x;

            lPower *= -1;
            rPower *= -1;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(lPower), Math.abs(rPower));
            if (max > 1.0) {
                lPower /= max;
                rPower /= max;
            }

            robot.leftMotor.setPower(lPower);
            robot.rightMotor.setPower(rPower);




            // right trigger is fire cannon
            if (gamepad1.right_trigger > 0.25) {
                robot.cannon.setPower(-1.0);
            } else {
                robot.cannon.setPower(0.0); // 0.8 seconds for raising cannon and 0.65 seconds for firing cannon
            }

            // cannon raise by right joystick
            robot.raiser.setPower(gamepad1.right_stick_y / 3.5);

// ToDO write new linear actuator code
            if (gamepad1.dpad_left) {
                beaconPosition = 0.0;
            } else if(gamepad1.dpad_right) {
                beaconPosition = 0.8;
            } else{
                beaconPosition = 0.52;
            }

            beaconPosition = Range.clip(beaconPosition, 0.0, 0.8);

            robot.pusher.setPosition(beaconPosition);

            // picker
            // Use y and a to raise and lower the picker
            if (gamepad1.y) {
                pickPosition = 0.2;
            } else if (gamepad1.a) {
                pickPosition = 0.75;
            } else{
                pickPosition = 0.52;
            }
            // Move servo to new position.
            pickPosition = Range.clip(pickPosition, 0.0, 0.7);
            robot.picker.setPosition(pickPosition);

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