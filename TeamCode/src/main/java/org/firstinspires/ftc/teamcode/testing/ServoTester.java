package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
@Disabled
public class ServoTester extends LinearOpMode {

    public Servo port0;
    public ServoImplEx port2;
    public Servo port4;
    public Servo port5;


    public AnalogInput encoder;

    public RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode(){

        port0 = hardwareMap.get(Servo.class, "portC0");
        port2 = hardwareMap.get(ServoImplEx.class, "portC2");
        port2.setPwmRange(new PwmControl.PwmRange(510, 2490));
        port4 = hardwareMap.get(Servo.class, "portC4");
        port5 = hardwareMap.get(Servo.class, "port5");

        encoder = hardwareMap.get(AnalogInput.class, "armEncoder");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");


        waitForStart();

        //Claw Close: 0.8, Open: 0.5 Port 0;
        // Pivot Port 2
        // Funnel Port 4
        // Funnel Claw: Port 5

        while(opModeIsActive()){
            if(gamepad1.a){
                port4.setPosition(0.2);
            }

            if(gamepad1.b){
                //pivot
                port4.setPosition(0.65);
            }

            if(gamepad1.dpad_left){
                port5.setPosition(0.5);
            }

            if(gamepad1.dpad_right){
                port5.setPosition(0);
            }

            if(gamepad1.x){
                port5.setPosition(1);
            }



            double angle = encoder.getVoltage() / 3.3 * 360;

            telemetry.addData("Encoder", angle);
            telemetry.addData("Distance", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }



    }
}
