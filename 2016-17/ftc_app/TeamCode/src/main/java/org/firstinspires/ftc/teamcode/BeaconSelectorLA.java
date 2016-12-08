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
//@Disabled
public class BeaconSelectorLA extends LinearOpMode {
    /* Declare OpMode members. */

    String MY_COLOR = "RED";
    String OPPONENT_COLOR = "BLUE";

    int whiteLightBrightness = 13;


    HardwareTwitchy robot = new HardwareTwitchy();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    ColorSensor beaconSensor;
    ColorSensor bottomSensor;

    //MOTOR VARIABLE:
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


    //COLOR SENSOR VARIABLE:
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float bottomHsvValues[] = {0F, 0F, 0F};


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        beaconSensor = hardwareMap.colorSensor.get("beaconColor");
        bottomSensor = hardwareMap.colorSensor.get("bottomColor");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");



        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //SET UP THE COLOR SENSORS
        beaconSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        bottomSensor.setI2cAddress(I2cAddr.create8bit(0x4c));

        //passive mode for beaconSensor;
        beaconSensor.enableLed(false);
        //active mode for bottomSensor for finding white line.
        bottomSensor.enableLed(true);



        //SET UP THE GYROSENSOR
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
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

        // TODO Write all code here

        while (opModeIsActive()) {

            findAndPressButtom();
            gyroDrive(DRIVE_SPEED, 50, 0);
            findAndPressButtom();



            idle();

        }

    }



    public void findAndPressButtom () throws InterruptedException{

        //drive foward at 0.2 speed until sees white line.
        do {
            robot.rightMotor.setPower(0.2);
            robot.leftMotor.setPower(0.2);

        }while(findBottomBrightness() <= whiteLightBrightness);

        stopAllMotors();

        telemetry.addData(">", "white!");
        telemetry.update();


        gyroHold(TURN_SPEED, 0, 0.5);



        //when in front of color sensor, first go back 1 cm to check color.
        gyroDrive(0.1,-1,0);

        //if find myColor, then hold position and drive forward to the next beacon.
        if(findBeaconColor().equals(MY_COLOR)){
            telemetry.addData(">", "Color Found!");

            //keep the current orientation for precision.
            gyroHold(TURN_SPEED, 0, 1);

            //drive foward to next beacon.
            //celebration
            gyroDrive(DRIVE_SPEED,10,0);
            gyroDrive(DRIVE_SPEED,-10,0);



            //if not myColor, then drive forward to the next color find color.
        }else{
            gyroDrive(0.1,15,0);
            if (findBeaconColor().equals(MY_COLOR)){
                telemetry.addData(">","Color Found!");
                //keep the current orientation for precision.
                gyroHold(TURN_SPEED,0,1);

                //press buttom here.
                //celebrate.
                gyroDrive(DRIVE_SPEED,10,0);
                gyroDrive(DRIVE_SPEED,-10,0);

            }
        }
    }

    //find the hue from beaconColor and print the hue to telemetry.
    //this returns the name of color: "BLUE" or "RED".
    public String findBeaconColor() {

        //return variable
        double BEACON_HUE = 0;
        String BEACON_COLOR = "";
        int redValue;
        int blueValue;


        redValue = beaconSensor.red();
        blueValue = beaconSensor.blue();



        if(Math.abs(redValue - blueValue) >= 3){
            if((redValue - blueValue) > 0){
                BEACON_COLOR = "RED";
            }else if ((redValue - blueValue) < 0){
                BEACON_COLOR = "BLUE";
            }
        }





        // send the info back to driver station using telemetry function.
        telemetry.addData("Clear", beaconSensor.alpha());
        telemetry.addData("Red  ", beaconSensor.red());
        telemetry.addData("Green", beaconSensor.green());
        telemetry.addData("Blue ", beaconSensor.blue());
        //telemetry.addData("Hue", beaconHsvValues[0]);
        telemetry.addData("Color", BEACON_COLOR);
        telemetry.update();

        return BEACON_COLOR;
    }



    //this returns the brightness of bottom color.
    //white line should give bright reflection.
    public double findBottomBrightness() {

        //return variable
        double BOTTOM_BRIGHTNESS = 0;



        // values is a reference to the hsvValues array.
        final float values[] = bottomHsvValues;

        //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        // convert the RGB values to HSV values.
//        Color.RGBToHSV(bottomSensor.red() * 8, bottomSensor.green() * 8, bottomSensor.blue() * 8, bottomHsvValues);

        //double BEACON_HUE is for returning the value of hue. each color has a unique hue.
        BOTTOM_BRIGHTNESS = bottomSensor.alpha();



        // send the info back to driver station using telemetry function.
        telemetry.addData("Bottom Clear", bottomSensor.alpha());
//        telemetry.addData("Red  ", bottomSensor.red());
//        telemetry.addData("Green", bottomSensor.green());
//        telemetry.addData("Blue ", bottomSensor.blue());
        telemetry.addData("Bottom Hue", bottomHsvValues[0]);
        telemetry.update();

        return BOTTOM_BRIGHTNESS;
    }




    /**
     * Method to drive on a fixed compass bearing (angle).
     * Move will stop:
     * 1) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param direction Direction (1 or -1). Positive is foward, Negative is backward.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDriveInfinite(double speed,
                          int direction,
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

//            // Determine new target position, and pass to motor controller
//            moveCounts = (int) (distance * COUNTS_PER_CENTIMETERS);
//            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
//            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

//            // Set Target and Turn On RUN_TO_POSITION
//            robot.leftMotor.setTargetPosition(newLeftTarget);
//            robot.rightMotor.setTargetPosition(newRightTarget);

//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            speed = speed * direction;

            robot.rightMotor.setPower(speed);
            robot.leftMotor.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (direction < 0)
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




                //telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

//            // Stop all motion;
//            robot.leftMotor.setPower(0);
//            robot.leftMotor.setPower(0);
//            robot.rightMotor.setPower(0);
//            robot.rightMotor.setPower(0);

//            // Turn off RUN_TO_POSITION
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void stopAllMotors (){
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
}


