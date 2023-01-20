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



    public DcMotorEx intakeTurret;
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

        intakeTurret = hardwareMap.get(DcMotorEx.class, "intakeTurret");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        armLeft = hardwareMap.get(Servo.class, "portC0");
        armRight = hardwareMap.get(Servo.class, "portC2");


        port4C = hardwareMap.get(Servo.class, "portC4");
        port5E = hardwareMap.get(Servo.class, "portE5");

        vertical = hardwareMap.get(DcMotorEx.class, "vertical_slide");
        flip = hardwareMap.get(ServoImplEx.class, "portC0");





        waitForStart();

        while(opModeIsActive()){

            //Turret Rotation Values
            // Considers the turret from the slide closest to the value

            //right, arm on left side = -214
            //left, arm on right side = 243


//            if(gamepad1.a){
//                armLeft.setPosition(0.67);
//                armRight.setPosition(1 - 0.67);
//            }
//
//            if(gamepad1.b){
//                armLeft.setPosition(1);
//                armRight.setPosition(0);
//            }
//
//            if(gamepad1.x){
//                claw.setPosition(1);
//            }
//
//            if(gamepad1.y){
//                claw.setPosition(0);
//            }
//
//            if (gamepad2.dpad_up) {
//                DESIRED = HIGH;
//                vertical.setTargetPosition(HIGH);
//                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                vertical.setPower(1);
//            }
//
//            if (gamepad2.dpad_left) {
//                DESIRED = MEDIUM;
//                vertical.setTargetPosition(MEDIUM);
//                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                vertical.setPower(1);
//            }
//
//            if (gamepad2.dpad_right) {
//                DESIRED = LOW;
//                vertical.setTargetPosition(LOW);
//                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                vertical.setPower(1);
//            }
//
//            if(gamepad2.a){
//                flip.setPosition(0.7);
//            }
//
//            if(gamepad2.b){
//                flip.setPosition(0.3);
//            }

//            turret.setPower(gamepad2.left_stick_y);

            turret.setPower(gamepad1.left_stick_y);

            vertical.setPower(gamepad1.right_stick_y);

            if(gamepad1.a){
                armRight.setPosition(servoPosition);
            }

            if(gamepad1.b){
                armLeft.setPosition(servoPosition);
            }



            if(gamepad1.b){
                port5E.setPosition(0.5);
            }


            telemetry.addData("Slide Position", vertical.getCurrentPosition());
            telemetry.addData("Turret Angle", (turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE));

            telemetry.update();



        }


    }
}
