package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes;

import androidx.appcompat.app.ActionBar;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
@Disabled
public class DoubleLinearSlideTesting extends OpMode {

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public PIDController liftController;



    public static double slideP = 0;
    public static double slideI = 0;
    public static double slideD = 0;


    public static double lifttargetPosition;

    public static double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.7017855748031 / 145.1; //2 * pi * spool radius (in) / motor CPR

    @Override
    public void init(){
        liftController = new PIDController(slideP, slideI, slideD);

        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");

        liftMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    @Override
    public void loop(){
        liftController.setPID(slideP, slideI, slideD);



        liftMotorOne.setPower(gamepad1.left_stick_y);
        liftMotorTwo.setPower(gamepad1.left_stick_y);




        telemetry.addLine();
        telemetry.addData("Lift Target", lifttargetPosition);
        telemetry.addData("Left Stick", gamepad1.left_stick_y);
        telemetry.addData("Right Stick", gamepad1.right_stick_y);
        telemetry.update();



    }


}
