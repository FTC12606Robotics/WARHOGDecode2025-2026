package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="MagSwitchTest", group="Examples")
public class MagSwitchTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        DigitalChannel magSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");

        // Set switch as input
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean isTriggered = !magSwitch.getState(); // true when triggered (depending on wiring)

            telemetry.addData("Is Triggered: ", isTriggered);
            telemetry.update();
        }
    }
}
