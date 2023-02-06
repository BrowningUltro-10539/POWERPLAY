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
public class DriveOpMode extends CommandOpMode {
    private Robot robot;
    private ElapsedTime timer;
    private double loopTime = 0;

    private GamepadEx driver2Ex;


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, false);
        robot.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        for(LynxModule module : robot.getControllers()){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        robot.intake.update(IntakeSubsystem.RotateState.INTAKE);
        robot.intake.update(IntakeSubsystem.ArmState.INTAKE);

    }


    @Override
    public void run(){
        if(timer == null){
            timer = new ElapsedTime();
            robot.reset();
        }

        robot.read();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        if(gamepad1.dpad_left){
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.OPEN_CLAW)));
        }

        if(gamepad1.dpad_right){
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.CLOSE_CLAW)));
        }

        if(gamepad1.x){
            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.DECIDE)),
                    new WaitCommand(100),
                    new LiftPositionCommand(robot.lift, 4, 2)

            ));
        }

        if(gamepad1.y){
            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.MEDIUM_POLE)),
                    new WaitCommand(100),
                    new LiftPositionCommand(robot.lift,17, 2)
                    ));

        }

        if(gamepad1.b){
            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.DECIDE)),
                    new WaitCommand(100),
                    new LiftPositionCommand(robot.lift, 25, 2)

            ));
        }

        if(gamepad1.a){
            schedule(new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.OPEN_CLAW)),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new LiftPositionCommand(robot.lift, 0, 2),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.INTAKE))
                        )


            ));
        }

        if(gamepad1.dpad_down){
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.LOW_POLE)));
        }









//        if(gamepad2.left_bumper){
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




        robot.intake.loop();
        robot.lift.loop();


        CommandScheduler.getInstance().run();

        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("Second Angle: ", robot.driveSubsystem.getPitch());
        telemetry.addData("Third Angle: ", robot.driveSubsystem.getYaw());
        loopTime = loop;
        telemetry.update();

        for(LynxModule module : robot.getControllers()){
            module.clearBulkCache();
        }
    }

    @Override
    public void reset(){
        CommandScheduler.getInstance().reset();
    }


}
