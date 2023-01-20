package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeServoTesting extends LinearOpMode {

    public Servo leftLinkage;
    public Servo rightLinkage;
    public Servo leftArm;
    public Servo rightArm;
    public Servo leftClaw;


    @Override
    public void runOpMode(){
        leftLinkage = hardwareMap.get(Servo.class, "PortSomething");
        rightLinkage = hardwareMap.get(Servo.class, "PortSomething");
        leftArm = hardwareMap.get(Servo.class, "PortSomething");
        rightArm = hardwareMap.get(Servo.class, "PortSomething");
        leftClaw = hardwareMap.get(Servo.class, "PortSomething");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                leftLinkage.setPosition(0.5);
            }

            if(gamepad1.b){
                rightLinkage.setPosition(0.5);
            }

            if(gamepad1.x){
                leftLinkage.setPosition(0);
            }

            if(gamepad1.y){
                rightLinkage.setPosition(0);
            }

            if(gamepad1.dpad_up){
                leftArm.setPosition(1);
            }

            if(gamepad1.dpad_down){
                rightArm.setPosition(0);
            }

            if(gamepad1.dpad_left){
                leftArm.setPosition(0);
            }

            if(gamepad1.dpad_right){
                rightArm.setPosition(1);
            }
        }
    }
}
