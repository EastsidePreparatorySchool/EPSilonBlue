package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareTwitchy
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null; //why?
    public DcMotor  rightMotor = null;
    public DcMotor cannon =null;
    public DcMotor raiser = null;
    public Servo pusher         = null;
    public Servo picker = null;


    public final static double pusher_home = 0; // set starting positoin
    public final static double pusher_MIN_RANGE  = 0; // lowest position
    public final static double pusher_MAX_RANGE  = 0.50;// highest posible

    public final static double picker_home = 0; // set starting positoin
    public final static double picker_MIN_RANGE  = 0; // lowest position
    public final static double picker_MAX_RANGE  = 0.50;// highest posible
    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTwitchy() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("motorLeft");
        rightMotor = hwMap.dcMotor.get("motorRight");
        cannon = hwMap.dcMotor.get("cannon");
        raiser = hwMap.dcMotor.get("raiser");
        pusher = hwMap.servo.get("pusher");
        picker = hwMap.servo.get("picker");

        //set the two backward motors to run in reverse
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        leftMotor.setPower(0.0);
        cannon.setPower(0.0);
        rightMotor.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cannon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        raiser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
//        arm = hwMap.servo.get("arm");
//        claw = hwMap.servo.get("claw");
//        arm.setPosition(ARM_HOME);
//        claw.setPosition(CLAW_HOME);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
