package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class IntakeTurretPIDTune extends OpMode {

    private PIDController controller;
    private PIDController turretController;
    private AnalogInput armEncoder;

    private PIDController liftController;

    public static double armP = 0.006, armI = 0, armD = 0.004;
    public static double pT = 0.0, iT = 0, dT = 0.0000;
    public static double f = 0.07;


    public static double slideP = 0, slideI = 0, slideD = 0;
    public static double kG = 0.045;

    public static double servoPosition = 0.5;

    public static int target = 0;
    public static int turretTarget = 0;
    public static int armTargetAngle = 0;

    public static double TURRET_TICKS_PER_REV = 1425.1;
    public static double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360;

    public static double INTAKE_TURRET_TICKS_PER_REV = 537.7;
    public static double INTAKE_TURRET_TICKS_PER_DEGREE = INTAKE_TURRET_TICKS_PER_REV / 360.0;

    public static double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 1.38952756 / 384.5;

    public static double ARM_TICKS_PER_DEGREE = 1425.1/360;


    private DcMotorEx slide_motor1;
    private DcMotorEx intakeTurret;

    private DcMotorEx liftMotor;


    private Servo leftArm;
    private Servo rightArm;
    private Servo flip;
    private Servo outtake_claw;

    private RevColorSensorV3 colorSensorOut;

    public Robot robot;

    public static double slideTarget = 0;



    @Override
    public void init(){
        controller = new PIDController(armP, armI, armD);
        turretController = new PIDController(pT, iT, dT);
        liftController = new PIDController(slideP, slideI, slideD);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        slide_motor1 = hardwareMap.get(DcMotorEx.class, "arm");
        armEncoder = hardwareMap.get(AnalogInput.class, "armEncoder");

        intakeTurret = hardwareMap.get(DcMotorEx.class, "intakeTurret");
        liftMotor = hardwareMap.get(DcMotorEx.class, "vertical_slide");


        leftArm = hardwareMap.get(Servo.class, "portC0");
        rightArm = hardwareMap.get(Servo.class, "portC2");
        flip = hardwareMap.get(Servo.class, "portC4");

        outtake_claw = hardwareMap.get(Servo.class, "portE5");

        colorSensorOut = hardwareMap.get(RevColorSensorV3.class, "colorSensorOut");


        slide_motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }
    @Override
    public void loop(){




        controller.setPID(armP, armI, armD);

        turretController.setPID(pT, iT, dT);

        liftController.setPID(slideP, slideI, slideD);

        double turretPos = slide_motor1.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;



        double intakeTurretPos = intakeTurret.getCurrentPosition() / INTAKE_TURRET_TICKS_PER_DEGREE;


        double pid = controller.calculate(turretPos, target);
        double turretPid = turretController.calculate(intakeTurretPos, turretTarget);


        double angle = slide_motor1.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;

        double ff = Math.cos(armTargetAngle / (1425.1 / 360)) * f;
        double armPID = controller.calculate(angle, armTargetAngle);

        double armPIDwF = armPID + ff;


        double liftPos = liftMotor.getCurrentPosition() * SLIDE_TICKS_PER_INCH;
        double liftPower = liftController.calculate(liftPos, slideTarget);
        double actualLiftPower = liftPower + kG;

        liftMotor.setPower(actualLiftPower);

        slide_motor1.setPower(armPIDwF);


        if(gamepad1.a){
            rightArm.setPosition(0.5);
        }

        if(gamepad1.b){
            rightArm.setPosition(servoPosition);
        }

        if(gamepad1.x){
            rightArm.setPosition(0.25);
        }


        if(!gamepad1.left_bumper){
            if(colorSensorOut.getDistance(DistanceUnit.CM) < 5){
                outtake_claw.setPosition(0);
            } else {
                outtake_claw.setPosition(0.5);
            }
        }

        if(gamepad1.dpad_left){
            flip.setPosition(0.2);
        }

        if(gamepad1.dpad_right){
            flip.setPosition(0.7);
        }

        if(gamepad1.dpad_down){
            outtake_claw.setPosition(0.5);
        }



        intakeTurret.setPower(turretPid);

        telemetry.addData("Arm Angle", angle);
        telemetry.addData("Turret Pos Angle", intakeTurretPos);
        telemetry.addData("Turret Pos Target", turretTarget);
        telemetry.addData("Target ", target);
        telemetry.addData("Power", armPIDwF);
        telemetry.addData("Distance", colorSensorOut.getDistance(DistanceUnit.CM));

        telemetry.addData("SLIDE POSITION", liftMotor.getCurrentPosition() * SLIDE_TICKS_PER_INCH);
        telemetry.addData("SLIDE DESIRED POSITION", slideTarget);


        telemetry.update();
    }
}
