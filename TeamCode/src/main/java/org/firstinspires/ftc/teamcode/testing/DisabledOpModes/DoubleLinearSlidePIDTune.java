package org.firstinspires.ftc.teamcode.testing.DisabledOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.List;

@Config
@TeleOp

public class DoubleLinearSlidePIDTune extends OpMode {

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public PIDController liftController;

    public Servo claw, leftArm, rightArm;
    public ServoImplEx rotate;



    public static double slideP = 0.24;
    public static double slideI = 0;
    public static double slideD = 0;
    public static double slideKg = 0.14;

    public static double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.764445002 / 145.1;

    public List<LynxModule> hubControllers;

    public static double targetPosition = 0;


    @Override
    public void init(){
        liftController = new PIDController(slideP, slideI, slideD);

        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");

        liftMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm = hardwareMap.get(Servo.class, "portC0");
        rightArm = hardwareMap.get(Servo.class, "portC2");

        claw = hardwareMap.get(Servo.class, "portC5");
        rotate = hardwareMap.get(ServoImplEx.class, "portC4");
        rotate.setPwmRange(new PwmControl.PwmRange(500, 2500));

        hubControllers = hardwareMap.getAll(LynxModule.class);

        for(LynxModule module : hubControllers){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop(){


        setArm(0.1);
        claw.setPosition(0.5);
        rotate.setPosition(0.26);

        liftController.setPID(slideP, slideI, slideD);

        double liftPosition = liftMotorOne.getCurrentPosition() * SLIDE_TICKS_PER_INCH;
        double liftPosition2 = liftMotorTwo.getCurrentPosition() * SLIDE_TICKS_PER_INCH;

        double pid = liftController.calculate(liftPosition, targetPosition);

        double liftPower = pid + slideKg;

        liftMotorOne.setPower(liftPower);
        liftMotorTwo.setPower(liftPower);



        telemetry.addData("Lift Position, Motor 1", liftPosition);
        telemetry.addData("Lift Position, Motor 2", liftPosition2);
        telemetry.addLine();
        telemetry.addData("Lift Target", targetPosition);
        telemetry.addData("Lift Power", liftPower);
        telemetry.update();

        for(LynxModule module : hubControllers){
            module.clearBulkCache();
        }

    }

    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }

}
