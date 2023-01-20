package org.firstinspires.ftc.teamcode.subsystem;//package org.firstinspires.ftc.teamcode.subsystem;
//
//import com.acmerobotics.dashboard.config.Config;
//
//
//import com.acmerobotics.roadrunner.profile.MotionProfile;
//import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
//import com.acmerobotics.roadrunner.profile.MotionState;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Config
//public class IntakeSubsystemV3 extends SubsystemBase {
//    public final MotorEx turret;
//    private final Servo leftArm;
//    private final Servo rightArm;
//    private final Servo claw;
//
//    private final RevColorSensorV3 sensor;
//    public int sensorColor;
//    public double sensorDistance;
//
//
//    public MotionProfile profile;
//    public MotionState currentState;
//    private final ElapsedTime timer;
//    private final ElapsedTime voltageTimer;
//
//    private PIDController controller;
//    private final VoltageSensor voltageSensor;
//
//    public static double P = 0.0252;
//    public static double I = 0.003;
//    public static double D = 0.0001;
//
//
//    private double voltage;
//
//    private double turretPosition = 0;
//    private double turretAngle = 0;
//    public double targetPosition = 0;
//    public double turretTargetAngle = 0;
//    private double liftTurretAngle = 0;
//    public double power;
//
//
//    public double autoAimAngle = 0;
//    public boolean autoAim = false;
//
//
//    private final double TURRET_TICKS_PER_REV = 537.7 / 0.5714285714285714;
//    private final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;
//
//    private final double TURRET_LEFT_SAFE_ANGLE = -40;
//    private final double TURRET_RIGHT_SAFE_ANGLE = 30;
//
//    public static double claw_open = 1.0;
//    public static double claw_close = 0.0;
//
//    public final double arm_up = 0.63;
//    public final double arm_transition = 0.8;
//    public final double arm_down = 0.97;
//
//    // Add servo positions for stack via an array
//    public final double[] fiveStack = new double[]{0.88, 0.91, 0.925, 0.945, 0.975};
//
//    public int offset = 0;
//
//    public IntakeState intakeState = IntakeState.INTAKE;
//    public ArmState armState = ArmState.INTAKE;
//
//    public LiftSubsystem.TurretState liftTurretState;
//
//    private boolean isAuto;
//
//    public enum IntakeState{
//        INTAKE,
//        TRANSITION_TO_DEPOSIT,
//        DEPOSIT
//    }
//
//    public enum ClawState {
//        OPEN,
//        CLOSED
//    }
//
//    public enum ArmState {
//        INTAKE,
//        TRANSITION,
//        DEPOSIT
//    }
//
//    public enum TurretState {
//        STRAIGHT,
//        ALIGNED,
//        AUTO_AIM
//    }
//
//    public IntakeSubsystemV3(HardwareMap hardwareMap, boolean isAuto){
//        this.turret = new MotorEx(hardwareMap, "intakeTurret");
//
//        this.leftArm = hardwareMap.get(Servo.class, "port1");
//        this.rightArm = hardwareMap.get(Servo.class, "port3");
//        this.claw = hardwareMap.get(Servo.class, "port5");
//
//
//        this.sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");
//
//
//        /* Utilize for turret */
//        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1,0), new MotionState(0,0), 30, 25);
//
//        this.timer = new ElapsedTime();
//        timer.reset();
//
//        this.voltageTimer = new ElapsedTime();
//        voltageTimer.reset();
//
//
//
//        this.controller = new PIDController(P, I, D);
//        controller.setPID(P, I, D);
//
//        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        this.voltage = voltageSensor.getVoltage();
//
//        this.isAuto = isAuto;
//    }
//
//    public void update(ClawState state){
//        switch(state){
//            case OPEN:
//                claw.setPosition(claw_open);
//                break;
//            case CLOSED:
//                claw.setPosition(claw_close);
//                break;
//        }
//    }
//
//    public void update(ArmState state){
//        switch (state){
//            case INTAKE:
//                 setArm(arm_down);
//                 armState = state;
//                 break;
//            case TRANSITION:
//                setArm(arm_transition);
//                armState = state;
//                break;
//            case DEPOSIT:
//                setArm(arm_up);
//                armState = state;
//                break;
//        }
//    }
//
//
//    public void update(TurretState state){
//        switch(state){
//            case STRAIGHT:
//                setAutoAim(false);
//                setTurretTargetAngle(3);
//                break;
//            case ALIGNED:
//                setAutoAim(false);
//                if(liftTurretState.equals(LiftSubsystem.TurretState.LEFT_POLE)){
//                    setTurretTargetAngle(-getLiftTurretAngle() - 5);
//                } else if(liftTurretState.equals(LiftSubsystem.TurretState.RIGHT_POLE)){
//                    setTurretTargetAngle(getLiftTurretAngle() - 5);
//                } else if(liftTurretState.equals(LiftSubsystem.TurretState.STRAIGHT)){
//                    setTurretTargetAngle(0);
//                } else if(liftTurretState.equals(LiftSubsystem.TurretState.NEG_NINETY)){
//                    setTurretTargetAngle(-10);
//                } else if(liftTurretState.equals(LiftSubsystem.TurretState.POS_NINETY)){
//                    setTurretTargetAngle(25);
//                }
//                break;
//            case AUTO_AIM:
//                autoAim = true;
//                break;
//        }
//
//
//    }
//
//    /* TODO:
//    *   Create commands using this intake state.
//    * */
//    public void update(IntakeState state){
//        intakeState = state;
//    }
//
//    public void loop() {
//
//        power = controller.calculate(turretAngle, turretTargetAngle) / voltage * 12;
//
//        if(autoAim && !isAuto){ setTurretTargetAngle(autoAimAngle); }
//
//
//            switch(intakeState){
//                case INTAKE:
//                    if(!isAuto) { if(sensor.getDistance(DistanceUnit.CM) < 1.5){
//                        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                                        new InstantCommand(() -> update(ClawState.CLOSED)),
//                                        new InstantCommand(() -> update(IntakeState.DEPOSIT))));
//                    } else {
//                        CommandScheduler.getInstance().schedule(new InstantCommand(() -> update(ClawState.OPEN)));
//                        CommandScheduler.getInstance().schedule(new InstantCommand(() -> update(ArmState.INTAKE)));
//                    } }
//                    break;
//                case TRANSITION_TO_DEPOSIT:
//                case DEPOSIT:
//                    autoAim = false;
//                    break;
//            }
//
//
//    }
//
//
//    public void read(){
//        turretPosition = turret.encoder.getPosition() + offset;
//        turretAngle = turret.encoder.getPosition() / TURRET_TICKS_PER_DEGREE;
//        sensorDistance = sensor.getDistance(DistanceUnit.CM);
//    }
//
//    public void write(){
//        turret.set(power);
//    }
//
//
//    public void setTurretTargetAngle(double angle){ turretTargetAngle = angle; }
//    public void setLiftTurretCurrentAngle(double angle){ liftTurretAngle = angle; }
//    public void setLiftTurretState(LiftSubsystem.TurretState state) { liftTurretState = state;  }
//    public void setAutoAimAngle(double angle){ autoAimAngle = angle; }
//    public void setAutoAim(boolean bool){ autoAim = bool; }
//
//    public int getPos(){ return (int) turretPosition; }
//    public double getTurretAngle() { return turretAngle; }
//    public double getTargetAngle() { return turretTargetAngle; }
//    public double getLiftTurretAngle() { return liftTurretAngle; }
//    public LiftSubsystem.TurretState getLiftTurretState(){ return liftTurretState; }
//
//
//
//
//    public void resetTimer(){
//        timer.reset();
//    }
//
//    public void newProfile(double targetPos, double max_v, double max_a){
//        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(targetPos, 0), max_v, max_a);
//        resetTimer();
//    }
//
//
//    public void setArm(double pos){
//        leftArm.setPosition(pos);
//        rightArm.setPosition(1 - pos);
//    }
//
//    public ArmState getArmState(){
//        return armState;
//    }
//
//}
