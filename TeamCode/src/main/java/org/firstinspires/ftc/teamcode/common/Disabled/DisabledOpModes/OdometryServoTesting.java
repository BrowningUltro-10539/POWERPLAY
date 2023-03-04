package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
@Disabled
public class OdometryServoTesting extends LinearOpMode {

    public Servo leftArm;
    public Servo rightArm;
    public Servo leftClaw;
    public Servo rotation;

    public Servo xPod;
    public Servo yPod;

    public DcMotorEx xPodMotor;
    public DcMotorEx yPodMotor;

    public static double ROTATION_INTAKE = 1;
    public static double ROTATION_OUTTAKE = 0.0;

    public static double CLAW_OPEN = 1;
    public static double CLAW_CLOSE = 0.5;

    public static double ARM_UP = 0;
    public static double ARM_DOWN = 1;

    public static double Y_POD_UP = 0.2;
    public static double Y_POD_DOWN = 0.9;

    public static double X_POD_DOWN = 0.7;
    public static double X_POD_UP = 0.03;

    @Override
    public void runOpMode(){
        leftArm = hardwareMap.get(Servo.class, "portC0");
        rightArm = hardwareMap.get(Servo.class, "portC2");
        leftClaw = hardwareMap.get(Servo.class, "portC5");
        rotation = hardwareMap.get(Servo.class, "portC4");

        yPod = hardwareMap.get(Servo.class, "yPod_rightServo");
        xPod = hardwareMap.get(Servo.class, "xPod_leftServo");

        yPodMotor = hardwareMap.get(DcMotorEx.class, "RB");
        xPodMotor = hardwareMap.get(DcMotorEx.class, "LT");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                yPod.setPosition(Y_POD_UP);
            }

            if(gamepad1.b){
                yPod.setPosition(Y_POD_DOWN);
            }

            if(gamepad1.y){
                xPod.setPosition(X_POD_UP);
            }

            if(gamepad1.x){
                xPod.setPosition(X_POD_DOWN);
            }
            telemetry.addData("X_POD", xPodMotor.getCurrentPosition());
            telemetry.addData("Y_POD", yPodMotor.getCurrentPosition());
            telemetry.update();
        }


    }

    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }
}
