package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp
@Config
public class PrimitiveMotorTester extends LinearOpMode {




    public Servo armLeft;
    public Servo armRight;
    public Servo claw;

    public DcMotorEx vertical;
    public DcMotorEx turret;
    public ServoImplEx flip;

    public Servo port4C;
    public Servo port5E;

    public static int HIGH = 1200;
    public static int MEDIUM = 700;
    public static int LOW = 0;
    public int DESIRED = 0;

    private final double TURRET_TICKS_PER_REV = 145.1 * 28;
    private final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

    public static double servoPosition = 0;


    @Override
    public void runOpMode(){


        turret = hardwareMap.get(DcMotorEx.class, "turret");
        armLeft = hardwareMap.get(Servo.class, "portC0");
        armRight = hardwareMap.get(Servo.class, "portC2");


        port4C = hardwareMap.get(Servo.class, "portC4");
        port5E = hardwareMap.get(Servo.class, "portE5");

        vertical = hardwareMap.get(DcMotorEx.class, "vertical_slide");
        flip = hardwareMap.get(ServoImplEx.class, "portC0");





        waitForStart();

        while(opModeIsActive()){



            turret.setPower(gamepad1.left_stick_y);

            vertical.setPower(gamepad1.right_stick_y);

            if(gamepad1.a){
                setArm(0.52);
            }

            if(gamepad1.b){
                setArm(1);
            }



            if(gamepad1.b){
                port5E.setPosition(0.5);
            }


            telemetry.addData("Slide Position", vertical.getCurrentPosition());
            telemetry.addData("Turret Angle", (turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE));

            telemetry.update();



        }


    }

    public void setArm(double pos){
        armLeft.setPosition(pos);
        armRight.setPosition(1 - pos);
    }
}
