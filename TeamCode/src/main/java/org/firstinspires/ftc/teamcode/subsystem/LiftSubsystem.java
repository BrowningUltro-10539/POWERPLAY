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
    public final MotorEx turret;
    public final Servo funnel;
    public final Servo outtakeClaw;



    private MotionProfile profile;
    public MotionState currentState;

    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController liftController;
    private final VoltageSensor voltageSensor;

    private final PIDController turretController;


    private double voltage;
    private double liftPosition;
    private int turretPosition;
    private double turretHeading;

    public static double P = 0.212;
    public static  double I = 0;
    public static double D = 0.00000;
    public static double Kg = 0.045;

    private final double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 1.38952756 / 384.5;

    private final double Pt = 0.1;
    private final double It = 0.0;
    private final double Dt = 0.0001;
    private final double Ft = 0.0001;

    private boolean isAuto = false;

    public double liftPower = 0.0;
    public double liftTargetPosition = 0.0;

    public double turretPower = 0.0;
    public double turretTargetPosition = 0.0;
    public double turretTargetAngle = 0.0;
    private final double TURRET_TICKS_PER_REV = 145.1 * 28;
    private final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

    public int offset = 0;

    public LiftState liftState = LiftState.READY_TO_INTAKE;
    public TurretState turretState = TurretState.STRAIGHT;

    public double funnelIntakePosition = 0.2;
    public double funnelTransitionPosition = 0.35;
    public double funnelDepositPosition = 0.65;

    public enum FunnelState {
        INTAKE,
        TRANSITION,
        OUTTAKE
    }

    public enum LiftState{
        READY_TO_INTAKE,
        HIGH_POLE_EXTEND,
        MEDIUM,

    }

    public enum TurretState {
        STRAIGHT,
        LEFT_POLE,
        RIGHT_POLE,
        NEG_NINETY,
        POS_NINETY
    }


    public LiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.lift1 = new MotorEx(hardwareMap, "vertical_slide");
        this.lift1.setInverted(true);
        this.turret = new MotorEx(hardwareMap, "turret");
        this.funnel = hardwareMap.get(Servo.class, "portC4");
        this.outtakeClaw = hardwareMap.get(Servo.class, "portE5");

        this.timer = new ElapsedTime();
        timer.reset();

        if(isAuto){
            lift1.encoder.reset();
            turret.encoder.reset();
        }

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.liftController = new PIDController(P, I, D);
        liftController.setPID(P, I, D);

        this.turretController = new PIDController(Pt, It, Dt);
        turretController.setPID(Pt, It, Dt);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        this.isAuto = isAuto;
    }

    public void loop() {
        liftPower = liftController.calculate(liftPosition, liftTargetPosition) + Kg;
        turretPower = turretController.calculate(turretHeading, turretTargetAngle) / voltage * 12;
    }


    public void update(FunnelState state){
        switch(state){
            case INTAKE:
                funnel.setPosition(funnelIntakePosition);
                break;
            case TRANSITION:
                funnel.setPosition(funnelTransitionPosition);
                break;
            case OUTTAKE:
                funnel.setPosition(funnelDepositPosition);
        }
    }

    public void update(TurretState state){
        switch(state){
            case STRAIGHT:
                setTargetTurretAngle(0);
                turretState = state;
                break;
            case LEFT_POLE:
                setTargetTurretAngle(-21.5);
                turretState = state;
                break;
            case RIGHT_POLE:
                setTargetTurretAngle(21.5);
                turretState = state;
                break;
            case POS_NINETY:
                setTargetTurretAngle(90);
                turretState = state;
                break;
            case NEG_NINETY:
                setTargetTurretAngle(-90);
                turretState = state;
                break;
        }
    }

    public void read(){
        liftPosition = lift1.encoder.getPosition() * SLIDE_TICKS_PER_INCH;
        turretPosition = turret.encoder.getPosition();
        turretHeading = turret.encoder.getPosition() / TURRET_TICKS_PER_DEGREE;
    }

    public void write(){
        lift1.set(liftPower);
        turret.set(turretPower);
    }

    public double getLiftPos(){
        return liftPosition;
    }
    public int getTurretPos(){ return turretPosition; }
    public double getTurretAngle(){ return getTurretPos() / TURRET_TICKS_PER_DEGREE; }


    public void setTargetLiftPosition(double value){ liftTargetPosition = value; }
    public void setTargetTurretPosition(double value) { turretTargetPosition = value; }
    public void setTargetTurretAngle(double angle) { turretTargetAngle = angle; }

    public void resetTimer() {
        timer.reset();
    }

    public void updateLiftState(LiftState state){
        liftState = state;
    }
    public void updateTurretState(TurretState state){
        turretState = state;
    }

    public LiftState getLiftState(){ return liftState; }
    public TurretState getTurretState(){ return turretState; }


    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getLiftPos(), 0), new MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }


}
