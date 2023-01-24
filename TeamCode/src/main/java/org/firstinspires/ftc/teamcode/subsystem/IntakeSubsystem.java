package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final Servo rotate;
    private final Servo claw;
    private final Servo leftArm;
    private final Servo rightArm;

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


    public static double CLAW_OPEN = 0;
    public static double CLAW_CLOSE = 0.5;

    public static double ROTATE_INTAKE = 0.98;
    public static double ROTATE_OUTTAKE = 0.3;
    public static double ROTATE_MID = 0.25;

    public static double ARM_DOWN = 1;
    public static double ARM_UP = 0.45;

    public DualAngle[] FIVE_STACK_POSITIONS = new DualAngle[]{
            new DualAngle(65,0.10),
            new DualAngle(60,0.10),
            new DualAngle(55,0.12),
            new DualAngle(35,0.10),
            new DualAngle(30,0.10)
    };

    private boolean isAuto;

    public enum IntakeState { INTAKE, DECIDE, TRANSFER, LOW_POLE, MANUAL, DEPOSIT, CLOSE_CLAW}
    public enum ArmState { INTAKE, TRANSITION,  DEPOSIT }
    public enum ClawState { OPEN, CLOSED }
    public enum RotateState { INTAKE, MID, TRANSFER }


    public IntakeState intakeState = IntakeState.INTAKE;

    public LiftSubsystem.TurretState liftTurretState;
    private double liftTurretAngle = 0;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.rotate = hardwareMap.get(Servo.class, "portC4");
        this.claw = hardwareMap.get(Servo.class, "portC5");

        this.leftArm = hardwareMap.get(Servo.class, "portC0");
        this.rightArm = hardwareMap.get(Servo.class, "portC2");

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

    public void update(RotateState state){
        switch (state){
            case INTAKE:
                setRotate(ROTATE_INTAKE);
                break;
            case MID:
                setRotate(ROTATE_MID);
                break;
            case TRANSFER:
                setRotate(ROTATE_OUTTAKE);
        }
    }


    public void update(ArmState state){
        switch(state){
            case INTAKE:
                setArm(ARM_DOWN);
                break;
            case TRANSITION:
                setArm(0.75);
                break;
            case DEPOSIT:
                setArm(ARM_UP);
                break;
        }
    }




    public void update(IntakeState state){
        switch(state){
            case INTAKE:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup()
                );
                intakeState = state;

                break;
            case DECIDE:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(100)
                        )
                );
                intakeState = state;
                break;
            case TRANSFER:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(20)
                        )
                );
                intakeState = state;
                break;

            case LOW_POLE:
                CommandScheduler.getInstance().schedule(
                        new ParallelCommandGroup(
                                new WaitCommand(30)
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
            case CLOSE_CLAW:
                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> setClaw(0.8))
                );
        }
    }

    public void read(){ colorSensorDistance = colorSensor.getDistance(DistanceUnit.CM); }

    public void loop(){
        if(colorSensor.getDistance(DistanceUnit.CM) < 4 && intakeState.equals(IntakeState.INTAKE)){
            update(IntakeState.DECIDE);
        }
    }

    public void write(){}


    public void setTurretTargetPosition(double position){ turretTargetPosition = position; }
    public void setTurretTargetAngle(double angle){ turretTargetAngle = angle; }
    public double getTurretAngle(){ return turretAngle; }

    public void setArmTargetPosition(double position){ armTargetPosition = position; }
    public void setArmTargetAngle(double angle){ armTargetAngle = angle; }
    public double getArmAngle(){ return armAngle; }


    public void setLiftTurretCurrentAngle(double angle){ liftTurretAngle = angle; }
    public void setLiftTurretState(LiftSubsystem.TurretState state) { liftTurretState = state;  }
    public double getLiftTurretAngle() { return liftTurretAngle; }



    public void setRotate(double pos){
        rotate.setPosition(pos);
    }
    public void setClaw(double pos){
        claw.setPosition(pos);
    }



    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }

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



}
