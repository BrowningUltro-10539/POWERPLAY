package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public final MotorEx turret;
    public final MotorEx arm;
    private final AnalogInput armEncoder;

    private final Servo pivot;
    private final Servo claw;

    /* New Intake Materials */
    //START HERE

    private final Servo leftLinkage;
    private final Servo rightLinkage;
    private final Servo leftArm;
    private final Servo rightArm;


    public static double ARM_DOWN = 0.0;
    public static double ARM_UP = 0.0;
    public static double ARM_TRANSFER = 0.0;


    //END HERE



    private final RevColorSensorV3 colorSensor;

    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;

    private PIDController turretController;
    private PIDController armController;


    public static double Pt = 0.03;
    public static double It = 0;
    public static double Dt = 0.0004;

    public static double Pa = 0.057;
    public static double Ia = 0.00001;
    public static double Da = 0.001;
    public static double Fa = 0.072;

    private final VoltageSensor voltageSensor;
    private double voltage;

    private double turretPosition = 0;
    private double turretAngle = 0;
    private double turretTargetPosition = 0;
    private double turretTargetAngle = 0;
    private double turretPower = 0;

    private double armPosition = 0;
    private double armAngle = 0;
    private double armAbsoluteAngle = 0;
    private double armTargetPosition = 0;
    private double armTargetAngle = 0;
    private double armPower = 0;
    private double position = 0;

    private double colorSensorDistance;


    public boolean autoAim = false;


    private final double TURRET_TICKS_PER_REV = 537.7;
    private final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

    private final double ARM_TICKS_PER_REV = 1425.1;
    private final double ARM_TICKS_PER_DEGREE = ARM_TICKS_PER_REV / 360.0;

    public static double CLAW_OPEN = 0.5;
    public static double CLAW_CLOSE = 0.8;

    public static double ARM_DOWN_ANGLE = 10;
    public static double ARM_UP_ANGLE = 90;

    public static double PIVOT_TRANSFER = 0.45;
    public static double PIVOT_INTAKE = 0.2;
    public static double PIVOT_MID = 0.30;

    public DualAngle[] FIVE_STACK_POSITIONS = new DualAngle[]{
            new DualAngle(65,0.10),
            new DualAngle(60,0.10),
            new DualAngle(55,0.12),
            new DualAngle(35,0.10),
            new DualAngle(30,0.10)
    };

    private boolean isAuto;

    public enum IntakeState { INTAKE, DECIDE, TRANSFER, LOW_POLE, MANUAL, DEPOSIT, OPEN_CLAW}
    public enum ArmState { INTAKE, TRANSITION,  DEPOSIT }
    public enum TurretState{ STRAIGHT, ALIGNED, AUTO_AIM }
    public enum ClawState { OPEN, CLOSED }
    public enum PivotState { INTAKE, MID, TRANSFER }
    public enum LinkageState {INTAKE, MID, TRANSFER }

    public IntakeState intakeState = IntakeState.INTAKE;

    public LiftSubsystem.TurretState liftTurretState;
    private double liftTurretAngle = 0;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.turret = new MotorEx(hardwareMap, "intakeTurret");
        this.arm = new MotorEx(hardwareMap, "arm");
        this.armEncoder = hardwareMap.get(AnalogInput.class, "armEncoder");

        this.pivot = hardwareMap.get(Servo.class, "portC2");
        this.claw = hardwareMap.get(Servo.class, "portC0");

        /* New Intake Stuff*/
        this.leftLinkage = hardwareMap.get(Servo.class, "PortSomething");
        this.rightLinkage = hardwareMap.get(Servo.class, "PortSomething");
        this.leftArm = hardwareMap.get(Servo.class, "PortSomething");
        this.rightArm = hardwareMap.get(Servo.class, "PortSomething");


        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        this.timer = new ElapsedTime();
        this.voltageTimer = new ElapsedTime();

        timer.reset();
        voltageTimer.reset();

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        this.turretController = new PIDController(Pt, It, Dt);
        turretController.setPID(Pt, It, Dt);

        this.armController = new PIDController(Pa, Ia, Da);
        armController.setPID(Pa, Ia, Da);

        this.isAuto = isAuto;
    }

    public void update(ClawState state){
        switch(state){
            case OPEN:
                setClaw(CLAW_OPEN);
                break;
            case CLOSED:
                setClaw(CLAW_CLOSE);
                break;
        }
    }

    public void update(PivotState state){
        switch (state){
            case INTAKE:
                setPivot(PIVOT_INTAKE);
                break;
            case MID:
                setPivot(PIVOT_MID);
                break;
            case TRANSFER:
                setPivot(PIVOT_TRANSFER);
        }
    }

    public void update(LinkageState state){
        switch(state){
            case INTAKE:
                setLinkage(0.0);
                break;
            case MID:
                setLinkage(0.1);
                break;
            case TRANSFER:
                setLinkage(0.2);
                break;
        }
    }
    public void update(ArmState state){
        switch(state){
            case INTAKE:
                setArmTargetAngle(ARM_DOWN_ANGLE);
                break;
            case TRANSITION:
                setArmTargetAngle((ARM_UP_ANGLE + ARM_DOWN_ANGLE) / 2);
                break;
            case DEPOSIT:
                setArmTargetAngle(ARM_UP_ANGLE);
                break;
        }
    }

    public void update(TurretState state){
        switch(state){
            case STRAIGHT:
                setTurretTargetAngle(0);
                break;
            case ALIGNED:
                if(liftTurretState.equals(LiftSubsystem.TurretState.LEFT_POLE)){
                    setTurretTargetAngle(-getLiftTurretAngle() - 5);
                } else if(liftTurretState.equals(LiftSubsystem.TurretState.RIGHT_POLE)){
                    setTurretTargetAngle(getLiftTurretAngle() - 5);
                } else if(liftTurretState.equals(LiftSubsystem.TurretState.STRAIGHT)){
                    setTurretTargetAngle(0);
                } else if(liftTurretState.equals(LiftSubsystem.TurretState.NEG_NINETY)){
                    setTurretTargetAngle(-10);
                } else if(liftTurretState.equals(LiftSubsystem.TurretState.POS_NINETY)){
                    setTurretTargetAngle(25);
                }
                break;
            case AUTO_AIM:
                autoAim = true;
                break;
        }
    }


    public void update(IntakeState state){
        switch(state){
            case INTAKE:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> setArmTargetAngle(18)),
                                new InstantCommand(() -> update(PivotState.INTAKE)),
                                new InstantCommand(() -> update(ClawState.OPEN))
                        )
                );
                intakeState = state;

                break;
            case DECIDE:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> update(ClawState.CLOSED)),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> setArmTargetAngle(90)),
                                        new InstantCommand(() -> update(PivotState.MID))
                                )
                        )
                );
                intakeState = state;
                break;
            case TRANSFER:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> setArmTargetAngle(105)),
                                new InstantCommand(() -> update(PivotState.TRANSFER)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> update(ClawState.OPEN)),
                                new WaitCommand(500),
                                new InstantCommand(() -> update(PivotState.MID)),
                                new InstantCommand(() -> setArmTargetAngle(80))
                        )
                );
                intakeState = state;
                break;

            case LOW_POLE:
                CommandScheduler.getInstance().schedule(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> setArmTargetAngle(90)),
                                new InstantCommand(() -> setPivot(0.2))
                        )
                );
                intakeState = state;
                break;
            case MANUAL:
                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> setClaw(0.5))
                );
                intakeState = state;
                break;
            case OPEN_CLAW:
                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> setClaw(0.8))
                );
        }
    }

    public void read(){
        turretPosition = turret.encoder.getPosition();
        turretAngle = turret.encoder.getPosition() / TURRET_TICKS_PER_DEGREE;
        armAbsoluteAngle = arm.encoder.getPosition() / ARM_TICKS_PER_DEGREE;
        colorSensorDistance = colorSensor.getDistance(DistanceUnit.CM);
    }

    public void loop(){
        turretPower = turretController.calculate(turretAngle, turretTargetAngle) / voltage * 12;
        armPower = armController.calculate(armAbsoluteAngle, armTargetAngle) + (Math.cos(armTargetAngle / armAbsoluteAngle) * Fa);

        armPower = armPower * (voltage / 12);


        if(colorSensor.getDistance(DistanceUnit.CM) < 4 && intakeState.equals(IntakeState.INTAKE)){
            update(IntakeState.DECIDE);
        }
    }

    public void write(){
        arm.set(armPower);
        turret.set(turretPower);
    }


    public void setTurretTargetPosition(double position){ turretTargetPosition = position; }
    public void setTurretTargetAngle(double angle){ turretTargetAngle = angle; }
    public double getTurretAngle(){ return turretAngle; }

    public void setArmTargetPosition(double position){ armTargetPosition = position; }
    public void setArmTargetAngle(double angle){ armTargetAngle = angle; }
    public double getArmAngle(){ return armAngle; }


    public void setLiftTurretCurrentAngle(double angle){ liftTurretAngle = angle; }
    public void setLiftTurretState(LiftSubsystem.TurretState state) { liftTurretState = state;  }
    public double getLiftTurretAngle() { return liftTurretAngle; }


    public static class DualAngle {
        private double armAngle = 0;
        private double pivotPosition = 0;

        public DualAngle(double armAngle, double pivotPosition){
            this.armAngle = armAngle;
            this.pivotPosition = pivotPosition;
        }

        public double getArmAngle() {
            return armAngle;
        }
        public double getPivotPosition(){
            return pivotPosition;
        }


    }

    public void setPivot(double pos){
        pivot.setPosition(pos);
    }

    public void setClaw(double pos){
        claw.setPosition(Range.clip(pos, 0.5, 1));
    }
    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }

    public void setLinkage(double pos){
        leftLinkage.setPosition(pos);
        rightLinkage.setPosition(pos);
    }





}
