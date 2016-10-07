package org.swerverobotics.library.examples;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.swerverobotics.library.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.*;

/**
 * An example that illustrates use of the telemetry dashboard and log in a synchronous OpMode
 */
@TeleOp(name="Telemetry (Synch)", group="Swerve Examples")
public class SynchTelemetryOp extends SynchronousOpMode
    {
    @Override protected void main() throws InterruptedException
        {
        final ElapsedTime elapsed = new ElapsedTime();

        this.telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        this.telemetry.log().setCapacity(10);             // We can control the number of lines used by the log

        // Wait until we've been given the ok to go. For fun, put out some
        // telemetry while we're doing so.
        while (!isStarted())
            {
            this.telemetry.addData("time", format(elapsed));
            this.telemetry.update();
            this.idle();
            }

        final int loopCountStart = getLoopCount();

        // Go go gadget robot!
        while (this.opModeIsActive())
            {
            if (this.updateGamepads())
                {
                // There is new gamepad input available. We choose to log some of its state.
                if (this.gamepad1.left_bumper)  this.telemetry.log().add(format(elapsed) + ": left bumper pressed");
                if (this.gamepad1.right_bumper) this.telemetry.log().add(format(elapsed) + ": right bumper pressed");
                }

            // Update the telemetry dashboard with fresh values
            this.telemetry.addData("time",  format(elapsed));
            this.telemetry.addData("count", format(getLoopCount() - loopCountStart));
            this.telemetry.addData("ms/loop", format(elapsed.milliseconds() / (getLoopCount() - loopCountStart)) + "ms");
            this.telemetry.addData("voltage", format(getBatteryVoltage()));

            // Update driver station and wait until there's something useful to do
            this.telemetry.update();
            this.idle();
            }
        }

    // A couple of handy functions for formatting data for the dashboard
    String format(ElapsedTime elapsed)
        {
        return String.format("%.1fs", elapsed.seconds());
        }
    String format(double d)
        {
        return String.format("%.1f", d);
        }
    String format(int i)
        {
        return String.format("%d", i);
        }

    // Compute the current battery voltage, just for fun
    double getBatteryVoltage()
        {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : this.hardwareMap.voltageSensor)
            {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
                {
                result = Math.min(result, voltage);
                }
            }
        return result;
        }
    }
