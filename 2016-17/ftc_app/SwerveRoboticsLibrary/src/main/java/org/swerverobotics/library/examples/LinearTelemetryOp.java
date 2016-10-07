package org.swerverobotics.library.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

/**
 * An example that illustrates use of the telemetry dashboard and log in a linear opmode
 */
@TeleOp(name="Telemetry (Linear)", group="Swerve Examples")
@Disabled
public class LinearTelemetryOp extends LinearOpMode
    {
    @Override public void runOpMode() throws InterruptedException
        {
        final ElapsedTime elapsed = new ElapsedTime();

        // Create a little helper utility that counts loops for us
        final IOpModeLoopCounter loopCounter = ClassFactory.createLoopCounter(this);

        try {
            this.telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
            this.telemetry.log().setCapacity(10);

            // Wait until we've been given the ok to go
            this.waitForStart();
            final int loopCountStart = loopCounter.getLoopCount();

            // Go go gadget robot!
            while (this.opModeIsActive())
                {
                if (this.gamepad1.left_bumper)  { this.telemetry.log().add(format(elapsed) + ": left bumper pressed");  while (opModeIsActive() && this.gamepad1.left_bumper) Thread.yield(); }
                if (this.gamepad1.right_bumper) { this.telemetry.log().add(format(elapsed) + ": right bumper pressed"); while (opModeIsActive() && this.gamepad1.right_bumper) Thread.yield(); }

                // Update the telemetry dashboard with fresh values
                this.telemetry.addData("time",  format(elapsed));
                this.telemetry.addData("count", loopCounter.getLoopCount() - loopCountStart);
                this.telemetry.addData("ms/loop", format(elapsed.milliseconds() / (loopCounter.getLoopCount() - loopCountStart)) + "ms");

                // Update driver station and wait until there's something useful to do
                this.telemetry.update();
                this.idle();
                }
            }
        finally
            {
            loopCounter.close();
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
    }
