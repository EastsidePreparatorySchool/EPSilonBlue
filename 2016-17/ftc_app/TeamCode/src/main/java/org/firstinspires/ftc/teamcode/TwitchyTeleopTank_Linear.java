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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Twitchy: Telop Tank", group="Twitchy")

public class TwitchyTeleopTank_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTwitchy   robot         = new HardwareTwitchy();            // Use a twitch's hardware
    //double          armPosition     = robot.ARM_HOME;                   // Servo safe position (current no servo. Exists for reference
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;     // sets rate to move servo

    private ElapsedTime runtime = new ElapsedTime();
    Timer timer =  new Timer();


    @Override
    public void runOpMode() throws InterruptedException {
        double vertical;
        double horizontol;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here 
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // done to check if error is in pp
            vertical = gamepad1.left_stick_y;
            horizontol = gamepad1.left_stick_x;

            if (vertical > 0.5 ){
                robot.rightMotor.setPower(-0.5);
                robot.leftMotor.setPower(-0.5);
            } else if (vertical <0.5){
                robot.leftMotor.setPower(0.5);
                robot.rightMotor.setPower(0.5);
            } else{
                robot.leftMotor.setPower(0.0);
                robot.rightMotor.setPower(0.0);
            }

            if (horizontol > 0.5) {
                robot.leftMotor.setPower(0.5);
                robot.rightMotor.setPower(-0.5);
            } else if (horizontol < 0.5) {
                robot.leftMotor.setPower(-0.5);
                robot.rightMotor.setPower(0.5);
            } else {
                robot.leftMotor.setPower(0.0);
                robot.rightMotor.setPower(0.0);
            }
            // disabled pp
//            double pp = Math.signum(vertical); // checking if it is positive and negative and setting power according to those variables)
//            robot.leftMotor.setPower(pp);
//            robot.rightMotor.setPower(pp);
//
//            pp = Math.signum(horizontol);
//            robot.rightMotor.setPower(pp);
//            robot.leftMotor.setPower(pp);


//            // Use gamepad Y & A raise and lower the arm
//            if (gamepad1.a)
//                armPosition += ARM_SPEED;
//            else if (gamepad1.y)
//                armPosition  -= ARM_SPEED;
//
//            // Use gamepad X & B to open and close the claw
//            if (gamepad1.x)
//                clawPosition += CLAW_SPEED;
//            else if (gamepad1.b)
//                clawPosition -= CLAW_SPEED;

            // if right trigger pressed will run cannon motor for half a second
            // currently cannon is not attached
            if (gamepad1.right_trigger > 0.25) {
                robot.cannon.setPower(0.33);// guess time
                timer.schedule(new TimerTask() {
                    public void run() {
                        robot.cannon.setPower(0.0);
                    }
                }, 500);
                }

//            // Move both servos to new position.
//            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
//            robot.arm.setPosition(armPosition);
//            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
//            robot.claw.setPosition(clawPosition);
//
//            // Send telemetry message to signify robot running;
//            telemetry.addData("arm",   "%.2f", armPosition);
//            telemetry.addData("claw",  "%.2f", clawPosition);
                telemetry.addData("horizontol", "%.2f", horizontol);
                telemetry.addData("vertical", "%.2f", vertical);
                telemetry.update();

                // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
                robot.waitForTick(40);
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        }
    }
}
