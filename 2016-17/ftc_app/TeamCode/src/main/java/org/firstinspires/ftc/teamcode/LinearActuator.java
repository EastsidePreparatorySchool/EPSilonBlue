package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by abedi on 12/10/2016.
 */
@Autonomous(name = "Linear Actuator", group = "Twitchy")
public class LinearActuator extends LinearOpMode {
    double beaconPosition;

    /* Declare OpMode members. */
    HardwareTwitchy robot = new HardwareTwitchy();   // Use a Pushbot's hardware

    //beacon selector variables:



    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

   waitForStart();

        while (opModeIsActive()) {
            beaconPosition = 0.2;
            beaconPosition = Range.clip(beaconPosition,0.2,0.8);
            robot.pusher.setPosition(beaconPosition);
            sleep(600);
            beaconPosition = 0.8;
            beaconPosition = Range.clip(beaconPosition,0.2,0.8);
            robot.pusher.setPosition(beaconPosition);
            sleep(1200);
            break;

        }
    }
}
