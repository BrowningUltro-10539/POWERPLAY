package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
@TeleOp
@Disabled
public class LinearSlidePIDTune extends OpMode {

    public DcMotorEx liftMotor;
    public PIDController liftController;

    public Servo leftArm, rightArm;

    public static double slideP = 0;
    public static double slideI = 0;
    public static double slideD = 0;
    public static double slideKg = 0.041;

    public static double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 1.38952756 / 384.5;

    public List<LynxModule> hubControllers;

    public static double targetPosition = 0;
    private final double TURRET_TICKS_PER_REV = 145.1 * 28;
    private final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;


    @Override
    public void init(){
        liftController = new PIDController(slideP, slideI, slideD);
        liftMotor = hardwareMap.get(DcMotorEx.class, "turret");

        leftArm = hardwareMap.get(Servo.class, "portC0");
        rightArm = hardwareMap.get(Servo.class, "portC2");

        hubControllers = hardwareMap.getAll(LynxModule.class);

        for(LynxModule module : hubControllers){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop(){

        setArm(0.52);

        liftController.setPID(slideP, slideI, slideD);





        double liftPosition = liftMotor.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;

        double pid = liftController.calculate(liftPosition, targetPosition);

        double liftPower = pid;

        liftMotor.setPower(liftPower);


        telemetry.addData("Lift Position", liftPosition);
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
