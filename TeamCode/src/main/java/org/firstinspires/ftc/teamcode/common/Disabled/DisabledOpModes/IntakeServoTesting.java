package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public class IntakeServoTesting extends LinearOpMode {

    public Servo leftArm;
    public Servo rightArm;
    public Servo leftClaw;
    public ServoImplEx rotation;

    public static double ROTATION_INTAKE = 1;
    public static double ROTATION_OUTTAKE = 0.0;

    public static double CLAW_OPEN = 1;
    public static double CLAW_CLOSE = 0.5;

    public static double ARM_UP = 0.6;
    public static double ARM_DOWN = 0.09;
    public static double ARM_DUNK = 0.58;

    @Override
    public void runOpMode(){
        leftArm = hardwareMap.get(Servo.class, "portC0");
        rightArm = hardwareMap.get(Servo.class, "portC2");
        leftClaw = hardwareMap.get(Servo.class, "portC5");
        rotation = hardwareMap.get(ServoImplEx.class, "portC4");
        rotation.setPwmRange(new PwmControl.PwmRange(500,2500));

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
