package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;

@Config
public class LiftSubsystem extends SubsystemBase {


    public final MotorEx lift1;
    public final MotorEx lift2;


    private MotionProfile profile;
    public MotionState currentState;

    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController liftController;
    private final VoltageSensor voltageSensor;


    private double voltage;
    private double liftPosition;


    public static double P = 0.335;
    public static  double I = 0;
    public static double D = 0;
    public static double Kg = 0.09;

    private final double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.7017855748031 / 145.1;


    private boolean isAuto = false;

    public double liftPower = 0.0;
    public double liftTargetPosition = 0.0;


    public int offset = 0;

    public LiftState liftState = LiftState.READY_TO_INTAKE;


    public enum LiftState{
        READY_TO_INTAKE,
        HIGH_POLE,
        MEDIUM_POLE,
        LOW_POLE

    }



    public LiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.lift1 = new MotorEx(hardwareMap, "liftMotorOne");
        this.lift2 = new MotorEx(hardwareMap, "liftMotorTwo");

        this.lift1.setInverted(true);
        this.lift2.setInverted(false);


        this.timer = new ElapsedTime();
        timer.reset();

        if(isAuto){
            lift1.encoder.reset();
        }

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.liftController = new PIDController(P, I, D);
        liftController.setPID(P, I, D);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        this.isAuto = isAuto;
    }

    public void loop() {
        liftPower = liftController.calculate(liftPosition, liftTargetPosition) + Kg;


//        if(voltageTimer.seconds() > 5){
//            voltage = voltageSensor.getVoltage();
//            voltageTimer.reset();
//        }
//
//        currentState = profile.get(timer.time());
//        if(currentState.getV() != 0){
//            liftTargetPosition = currentState.getX();
//        }
//
//        liftPower = (liftController.calculate(liftPosition, liftTargetPosition) + Kg) / (voltage * 14);


    }



    public void read(){ liftPosition = lift1.encoder.getPosition() * SLIDE_TICKS_PER_INCH; }

    public void write(){
        lift1.set(liftPower);
        lift2.set(liftPower);
    }

    public double getLiftPos(){
        return liftPosition;
    }
    public void setTargetLiftPosition(double value){ liftTargetPosition = value; }


    public void resetTimer() {
        timer.reset();
    }


    public LiftState getLiftState(){ return liftState; }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getLiftPos(), 0), new MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }




}
