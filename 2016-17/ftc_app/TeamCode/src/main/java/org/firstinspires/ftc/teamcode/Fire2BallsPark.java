package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by abedi on 12/1/2016.
 */

@Autonomous(name="Fire 2 and Park", group="Twitchy")
//@Disabled
public class Fire2BallsPark extends LinearOpMode {


    /**
     * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
     * It uses the common Pushbot hardware class to define the drive on the robot.
     * The code is structured as a LinearOpMode
     * <p>
     * The code REQUIRES that you DO have encoders on the wheels,
     * otherwise you would use: PushbotAutoDriveByTime;
     * <p>
     * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
     * otherwise you would use: PushbotAutoDriveByEncoder;
     * <p>
     * This code requires that the drive Motors have been configured such that a positive
     * power command moves them forward, and causes the encoders to count UP.
     * <p>
     * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
     * <p>
     * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
     * This is performed when the INIT button is pressed on the Driver Station.
     * This code assumes that the robot is stationary when the INIT button is pressed.
     * If this is not the case, then the INIT should be performed again.
     * <p>
     * Note: in this example, all angles are referenced to the initial coordinate frame set during the
     * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
     * <p>
     * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
     * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
     * This is consistent with the FTC field coordinate conventions set out in the document:
     * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
     * <p>
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */



        /* Declare OpMode members. */
    HardwareTwitchy robot = new HardwareTwitchy();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    ColorSensor beaconSensor;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP (try 1.0)
    static final double WHEEL_DIAMETER_CENTIMETERS = 10.0;     // For figuring circumference
    static final double COUNTS_PER_CENTIMETERS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTIMETERS * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.5;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.4;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.03;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.03;     // Larger is more responsive, but also less stable


    //beacon selector variables:
    String MY_COLOR = "RED";
    String OPPONENT_COLOR = "BLUE";
    int lightDetected = 2;
    double LEFT_POS = 0.5;
    double RIGHT_POS = 0.2;
    double MIDDLE_POS = 0.35;
    double CANNON_POS = 0.8;
    boolean flag = true;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        beaconSensor = hardwareMap.colorSensor.get("beaconColor");


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //passive mode for beacon color sensor;
        beaconSensor.enableLed(false);

        //set position of the pusher servo
        robot.pusher.setPosition(MIDDLE_POS);

        //set position of the picker servo. This position is up.
        robot.picker.setPosition(0.5);


        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();

        }
        gyro.resetZAxisIntegrator();


        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        // TODO Write all code here

        while (opModeIsActive()) {
            gyroDrive(DRIVE_SPEED, 45, 0.0);      //forward 45 cm
            //shoot;
            //shoot;
            robot.picker.setPosition(0.55);// middle position
            robot.raiser.setPower(-0.25);
            sleep(878);// preliminary things to shoot(raising cannon. moving things out of way)

            robot.raiser.setPower(-0.20);
            robot.cannon.setPower(-1.0);
            sleep(650);// shooting of the cannon

            robot.cannon.setPower(0.0);//lower cannon
            sleep(1000);

            // reset robot to start cannon fire position
            robot.raiser.setPower(0.0);
            robot.picker.setPosition(0.2);// also loads the cannon
            sleep(878);
            //shooting again robot pusher no longer attached and needs to be controlled like this
            //robot.pusher.setPosition(0.8);
            robot.picker.setPosition(0.75);// bottom position
            sleep(500);
            robot.raiser.setPower(-0.25);
            sleep(878);// preliminary things to shoot(raising cannon. moving things out of way)

            robot.raiser.setPower(-0.20);
            robot.cannon.setPower(-1.0);
            sleep(650);// shooting of the cannon

            robot.cannon.setPower(0.0);//lower cannon
            sleep(1000);

            // reset robot to start position
            robot.raiser.setPower(0.0);
            robot.picker.setPosition(0.2);
            //robot.pusher.setPosition(0.35);


            gyroDrive(DRIVE_SPEED,87,0.0);// to hit cap ball ( 15 second delay)

            // TODO stop writing code
            telemetry.addData("Path", "Complete");
            telemetry.update();
            break;
        }

    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in centimeters) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) throws InterruptedException {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_CENTIMETERS);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightMotor.setPower(speed);
            robot.leftMotor.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
//                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftBackMotor.getCurrentPosition(),
//                                                             robot.rightBackMotor.getCurrentPosition());
//                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("Counts Per Centimeters", COUNTS_PER_CENTIMETERS);

                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */
    public void gyroTurn(double speed, double angle)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    public void gyroHold(double speed, double angle, double holdTime)
            throws InterruptedException {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            idle();
        }

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}
