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
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

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
                    new NewLiftPositionCommand(robot.lift, 6, 200, 200, 2)

            ));
        }

        if(gamepad1.y){
            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.MEDIUM_POLE)),
                    new WaitCommand(100),
                    new NewLiftPositionCommand(robot.lift,12, 200 ,200, 2)
                    ));

        }

        if(gamepad1.b){
            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.DECIDE)),
                    new WaitCommand(100),
                    new NewLiftPositionCommand(robot.lift, 21.5, 200, 200, 2)

            ));
        }

        if(gamepad1.a){
            schedule(new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.OPEN_CLAW)),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new NewLiftPositionCommand(robot.lift, -0.09, 200, 200, 2),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.INTAKE))
                        )


            ));
        }


        if(gamepad1.left_trigger > 0.25){
            robot.lift.setSlideFactor(-1);
        } else if(gamepad1.right_trigger > 0.25){
            robot.lift.setSlideFactor(1);
        }


        schedule(new MecanumDriveCommand(robot.driveSubsystem, () -> gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> -gamepad1.right_stick_x));




        robot.intake.loop();
        robot.lift.loop();


        CommandScheduler.getInstance().run();

        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("Lift Position,", robot.lift.getLiftPos());
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
