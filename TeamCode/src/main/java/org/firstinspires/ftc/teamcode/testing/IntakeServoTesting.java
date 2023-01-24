package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class IntakeServoTesting extends LinearOpMode {

    public Servo leftArm;
    public Servo rightArm;
    public Servo leftClaw;
    public Servo rotation;

    public static double ROTATION_INTAKE = 1;
    public static double ROTATION_OUTTAKE = 0.0;

    public static double CLAW_OPEN = 0.5;
    public static double CLAW_CLOSE = 0;

    public static double ARM_UP = 0;
    public static double ARM_DOWN = 1;

    @Override
    public void runOpMode(){
        leftArm = hardwareMap.get(Servo.class, "portC0");
        rightArm = hardwareMap.get(Servo.class, "portC2");
        leftClaw = hardwareMap.get(Servo.class, "portC5");
        rotation = hardwareMap.get(Servo.class, "portC4");

        waitForStart();

        while(opModeIsActive()){



            if(gamepad1.a){
                rotation.setPosition(ROTATION_INTAKE);
            }

            if(gamepad1.b){
                rotation.setPosition(ROTATION_OUTTAKE);
            }

            if(gamepad1.x){
                leftClaw.setPosition(CLAW_OPEN);
            }

            if(gamepad1.y){
                leftClaw.setPosition(CLAW_CLOSE);
            }

            if(gamepad1.dpad_up){
                setArm(ARM_UP);
            }

            if(gamepad1.dpad_down){
                setArm(ARM_DOWN);
            }


        }
    }

    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }
}
