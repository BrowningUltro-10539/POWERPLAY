package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp
@Config
public class TestingSlideStackPositions extends CommandOpMode {
    private Robot robot;
    private ElapsedTime timer;
    private double loopTime = 0;

    private GamepadEx driver2Ex;

    public static double SLIDE_POS = 0;


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, false);
        robot.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        for(LynxModule module : robot.getControllers()){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }



    }


    @Override
    public void run(){
        if(timer == null){
            timer = new ElapsedTime();
            robot.reset();
        }

        robot.read();

        if(gamepad1.dpad_left){
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)));
        }

        if(gamepad1.dpad_right){
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)));
        }

        if(gamepad1.b){
            schedule(new LiftPositionCommand(robot.lift, SLIDE_POS, 2));
        }

        if(gamepad1.a){
            schedule(new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(200),
                        new LiftPositionCommand(robot.lift, 0, 2),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE))

                        )

            ));
        }


        if(gamepad1.right_bumper){
            schedule(new MecanumDriveCommand(robot.driveSubsystem, () -> -gamepad1.left_stick_y * 0.5, () -> -gamepad1.left_stick_x * 0.5, () -> gamepad1.right_stick_x * 0.5));
        } else {
            schedule(new MecanumDriveCommand(robot.driveSubsystem, () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x, () -> gamepad1.right_stick_x));
        }

//        if(gamepad2.left_bumper){w
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT)));
//        }
//
//
//        if(gamepad2.a){
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.RIGHT_POLE)));
//        }
//
//        if(gamepad2.b){
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.LEFT_POLE)));
//        }
//
//
//        if(gamepad2.dpad_up){
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.LiftState.HIGH_POLE_EXTEND)));
//        }
//
//        if(gamepad2.dpad_down){
//            schedule(new LiftPositionCommand(robot.lift, 0, 2));
//        }
//
//        if(gamepad2.dpad_left){
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.LiftState.MEDIUM)));
//        }


        schedule(new MecanumDriveCommand(robot.driveSubsystem, () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x, () -> gamepad1.right_stick_x));


        robot.intake.setLiftTurretState(robot.lift.turretState);
        robot.intake.setLiftTurretCurrentAngle(robot.lift.getTurretAngle());

        robot.intake.loop();
        robot.lift.loop();


        CommandScheduler.getInstance().run();

        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        for(LynxModule module : robot.getControllers()){
            module.clearBulkCache();
        }
    }

    @Override
    public void reset(){
        schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT)));
        CommandScheduler.getInstance().reset();
    }


}
