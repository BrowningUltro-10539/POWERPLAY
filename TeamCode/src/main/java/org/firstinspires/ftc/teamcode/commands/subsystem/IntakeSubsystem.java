package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private final Servo leftArm;
    private final Servo rightArm;
    private final ServoImplEx rotate;
    private final Servo claw;



    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;


    private final VoltageSensor voltageSensor;
    private double voltage;


    public static double CLAW_OPEN = 1;
    public static double CLAW_CLOSE = 0.5;

    public static double ROTATE_INTAKE = 1;
    public static double ROTATE_OUTTAKE = 0.0;
    public static double ROTATE_MID = 0.0;

    public static double ARM_DOWN = 0.09;
    public static double ARM_UP = 0.6;
    public static double ARM_DUNK = 0.58;
    public static double LOW_POLE = 0.0;
    public static double ARM_MID = 0.4;


    private boolean isAuto;

    public enum IntakeState { INTAKE, DECIDE, OPEN_CLAW, CLOSE_CLAW, LOW_POLE, MEDIUM_POLE}
    public enum ArmState { INTAKE, DEPOSIT, DUNK, MIDWAY, LOW_POLE, AUTO_INIT, POLE_DUNK}
    public enum ClawState { OPEN, CLOSED }
    public enum RotateState { INTAKE, MID, TRANSFER }

    public IntakeState intakeState = IntakeState.INTAKE;


    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){

        this.rotate = hardwareMap.get(ServoImplEx.class, "portC4");
        this.rotate.setPwmRange(new PwmControl.PwmRange(500, 2500));

        this.claw = hardwareMap.get(Servo.class, "portC5");

        this.leftArm = hardwareMap.get(Servo.class, "portC0");
        this.rightArm = hardwareMap.get(Servo.class, "portC2");



        this.timer = new ElapsedTime();
        this.voltageTimer = new ElapsedTime();

        timer.reset();
        voltageTimer.reset();

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

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
            case DEPOSIT:
                setArm(ARM_UP);
                break;
            case DUNK:
                setArm(ARM_DUNK);
                break;
            case LOW_POLE:
                setArm(LOW_POLE);
                break;
            case AUTO_INIT:
                setArm(0.58);
                break;
            case MIDWAY:
                setArm(ARM_MID);
                break;
            case POLE_DUNK:
                setArm(0.65);
                break;
        }
    }

    public void update(IntakeState state){
        if(!isAuto) {
            switch (state) {
                case INTAKE:
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> update(ClawState.OPEN)),
                                    new WaitCommand(200),
                                    new ParallelCommandGroup(
                                            new InstantCommand(() -> update(RotateState.INTAKE)),
                                            new InstantCommand(() -> update(ArmState.INTAKE))

                                    )
                            )
                    );
                    intakeState = state;

                    break;
                case DECIDE:
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> update(ClawState.CLOSED)),
                                    new WaitCommand(300),
                                    new ParallelCommandGroup(
                                            new InstantCommand(() -> update(RotateState.TRANSFER)),
                                            new InstantCommand(() -> update(ArmState.DEPOSIT))

                                    )
                            )
                    );
                    intakeState = state;
                    break;
                case OPEN_CLAW:
                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> update(ClawState.OPEN))
                    );
                    intakeState = state;
                    break;
                case CLOSE_CLAW:
                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> update(ClawState.CLOSED))
                    );
                    intakeState = state;
                    break;
                case LOW_POLE:
                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> update(ArmState.LOW_POLE)),
                            new InstantCommand(() -> update(RotateState.INTAKE))

                    );
                    break;
                case MEDIUM_POLE:
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                                    new InstantCommand(() -> update(ArmState.DUNK)),
                                    new InstantCommand(() -> update(ClawState.CLOSED)),
                                    new InstantCommand(() -> update(RotateState.TRANSFER))
                    )

                    );
                    break;
            }
        }
    }

    public void read(){}
    public void write(){}
    public void loop(){}


    public void setRotate(double pos){ rotate.setPosition(pos); }
    public void setClaw(double pos){
        claw.setPosition(pos);
    }

    public void setArm(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }




}
