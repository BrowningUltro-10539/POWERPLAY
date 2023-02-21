package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LiftSubsystem extends SubsystemBase {


    public final MotorEx lift1;
    public final MotorEx lift2;
    public final MotorEx liftEncoder;

    private MotionProfile profile;
    public MotionState currentState;

    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController liftController;
    private final VoltageSensor voltageSensor;


    private double voltage;
    private double liftPosition;


    public static double P = 0.26;
    public static  double I = 0;
    public static double D = 0;
    public static double kG = 0.21;

    private final double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.764445002 / 145.1;

    private boolean isAuto = false;

    public double liftPower = 0.0;
    public double liftTargetPosition = 0.0;

    public int offset = 0;


    public LiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.lift1 = new MotorEx(hardwareMap, "liftMotorOne");
        this.lift2 = new MotorEx(hardwareMap, "liftMotorTwo");
        this.liftEncoder = new MotorEx(hardwareMap, "LB");

        this.lift1.setInverted(false);
        this.lift2.setInverted(true);
        this.liftEncoder.setInverted(false);



        this.timer = new ElapsedTime();
        timer.reset();

        if(isAuto){
            liftEncoder.encoder.reset();
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
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        currentState = profile.get(timer.time());
        if (currentState.getV() != 0) {
            liftTargetPosition = currentState.getX();
        }

        liftPower = (liftController.calculate(liftPosition, liftTargetPosition) + kG)/ voltage * 14;

    }

    public void setSlideFactor(double factor) {
        double slideAddition = 1 * factor;
        double newPosition = liftPosition + slideAddition;
        if (currentState.getV() == 0) {
            liftTargetPosition = newPosition;
        }

    }
    public void read(){ liftPosition = liftEncoder.encoder.getPosition() * SLIDE_TICKS_PER_INCH; }

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

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getLiftPos(), 0), new MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }




}
