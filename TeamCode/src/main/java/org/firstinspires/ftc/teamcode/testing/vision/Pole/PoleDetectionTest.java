package org.firstinspires.ftc.teamcode.testing.vision.Pole;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class PoleDetectionTest extends LinearOpMode {

    PoleDetector d;

    @Override
    public void runOpMode() throws InterruptedException {

        d = new PoleDetector(this,telemetry);

        d.findPole();


        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {}

    }
}
