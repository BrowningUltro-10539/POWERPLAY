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

    public static double ROTATION_SERVO_POSITION = 0;

    @Override
    public void runOpMode(){
        leftArm = hardwareMap.get(Servo.class, "PortSomething");
        rightArm = hardwareMap.get(Servo.class, "PortSomething");
        leftClaw = hardwareMap.get(Servo.class, "PortSomething");
        rotation = hardwareMap.get(Servo.class, "PortSomething");

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.dpad_up){
                setArm(0.5);
            }

            if(gamepad1.dpad_down){
                setArm(0.7);
            }

            if(gamepad1.dpad_left){
                setArm(1);
            }

            if(gamepad1.dpad_right){
                setArm(0);
            }


            if(gamepad1.a){
                rotation.setPosition(ROTATION_SERVO_POSITION);
            }


        }
    }

    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }
}
