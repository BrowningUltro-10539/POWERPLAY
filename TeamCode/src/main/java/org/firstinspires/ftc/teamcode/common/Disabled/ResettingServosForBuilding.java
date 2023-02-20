package org.firstinspires.ftc.teamcode.common.Disabled;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config

public class ResettingServosForBuilding extends OpMode {
    public Servo leftArm;
    public Servo rightArm;

    public ServoImplEx rotate;
    public Servo claw;

    public static double ARM_POSITION = 0;
    public static double ROTATE_POSITION = 0;
    public static double CLAW_POSITION = 0;

    @Override
    public void init(){
        leftArm = hardwareMap.get(Servo.class, "portC0");
        rightArm = hardwareMap.get(Servo.class, "portC2");

        rotate = hardwareMap.get(ServoImplEx.class, "portC4");
        claw = hardwareMap.get(Servo.class, "portC5");

        rotate.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void loop(){
        if(gamepad1.dpad_left){
            setArm(ARM_POSITION);
        }

        if(gamepad1.right_bumper){
            claw.setPosition(CLAW_POSITION);
        }
        
        telemetry.addLine("ALL SERVOS ARE SET TO POSITION 0.5 FOR INITIAL CHECK");
        telemetry.addLine("TO RUN SERVOS MANUAL: ");
        telemetry.addLine("A: (IS SUPPOSED TO BE) LEFT ARM");
        telemetry.addLine("B: (IS SUPPOSED TO BE) RIGHT ARM");
        telemetry.addLine("X: (IS SUPPOSED TO BE) ROTATION");
        telemetry.addLine("Y: (IS SUPPOSED TO BE) CLAW");
        telemetry.addLine();
        telemetry.addLine("DPAD-UP: (IS SUPPOSED TO BE) ENTIRE ARM GOES UP (AKA OUTTAKE)");
        telemetry.addLine("DPAD-DOWN: (IS SUPPOSED TO BE) ENTIRE ARM GOES DOWN (AKA INTAKE)");
        telemetry.addLine();
        telemetry.addLine("TO BE USED WITH FTC DASHBOARD TUNING PROCESS ONLY!!!!!!!!");
        telemetry.addLine("DPAD-LEFT: (IS SUPPOSED TO BE) ARM DASHBOARD-SET VALUE");
        telemetry.addLine("DPAD-RIGHT: (IS SUPPOSED TO BE) ROTATE DASHBOARD-SET VALUE");
        telemetry.addLine("RIGHT_BUMPER: (IS SUPPOSED TO BE) CLAW DASHBOARD-SET VALUE");
        telemetry.update();
    }

    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }
}
