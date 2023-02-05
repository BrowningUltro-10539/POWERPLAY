package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.AutoCycleCommandV5;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.openftc.easyopencv.OpenCvCamera;

import java.nio.file.Watchable;

@Autonomous
@Config
public class RED_ALLIANCE_RIGHT_VERT_TESTING extends LinearOpMode {
    private Robot robot;
    private ElapsedTime timer;
    private double loopTime = 0;

    private OpenCvCamera camera;




    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, true);

        //

        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(270));
        robot.driveSubsystem.setPoseEstimate(startPose);

        TrajectorySequence testing = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                //Dropping off preload to pole
                .lineTo(new Vector2d(36, -24))
                .splineTo(new Vector2d(36, -8), Math.toRadians(145))
                .build();

        TrajectorySequence toConeStackAfterPreload = robot.driveSubsystem.trajectorySequenceBuilder(testing.end())
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .build();

        TrajectorySequence toPoleAfterConeOnePickUp = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
                .setReversed(true)
                //Going to pole
                .splineTo(new Vector2d(36, -8), Math.toRadians(145))
                .build();

        TrajectorySequence toConeTwoAfterConeOneDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeOnePickUp.end())
                .setReversed(false)
                .waitSeconds(1.5)
                //Going back to cone for Cycle 2
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .build();

        TrajectorySequence toPoleAfterConeTwoPickUp = robot.driveSubsystem.trajectorySequenceBuilder(toConeTwoAfterConeOneDeposit.end())
                .setReversed(true)
                //Going to pole
                .splineTo(new Vector2d(36, -8), Math.toRadians(145))
                .build();

        TrajectorySequence toConeThreeAfterConeTwoDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeTwoPickUp.end())
                .setReversed(false)
                .waitSeconds(1.5)
                //Going back to cone for Cycle 2
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .build();


        TrajectorySequence testing1 = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                //Dropping off preload to pole
                .lineTo(new Vector2d(36, -24))
                .splineTo(new Vector2d(34, -7), Math.toRadians(136))
                .waitSeconds(1.5)
                // Going to cones
                //CYCLE 1
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .waitSeconds(1.5)
                .setReversed(true)
                //Going to pole
                .splineTo(new Vector2d(34, -7), Math.toRadians(136))
                .setReversed(false)
                .waitSeconds(1.5)
                //Going back to cone for Cycle 2
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .waitSeconds(1.5)
                .setReversed(true)
                //Going to pole
                .splineTo(new Vector2d(34,-7), Math.toRadians(136))
                .setReversed(false)
                .waitSeconds(1.5)
                //Going back to cone for Cycle 3
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .waitSeconds(1.5)
                .setReversed(true)
                //Going to pole
                .splineTo(new Vector2d(34, -7), Math.toRadians(137))
                .setReversed(false)
                .waitSeconds(1.5)
                //Going back to cone for Cycle 4
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .waitSeconds(1.5)
                .setReversed(true)
                //Going to pole
                .splineTo(new Vector2d(34, -7), Math.toRadians(137))
                .setReversed(false)
                .waitSeconds(1.5)
                //Going back to cone for cycle 5
                .splineTo(new Vector2d(62.5, -11), Math.toRadians(0))
                .waitSeconds(1.5)
                .setReversed(true)
                //Going to pole
                .splineTo(new Vector2d(34, -7), Math.toRadians(137))
                .setReversed(false)
                .waitSeconds(1.5)

                .build();



        while(!isStarted()) {
            robot.read();

            for(LynxModule module : robot.getControllers()){
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }


            robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
            robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
            sleep(2000);
            robot.intake.update(IntakeSubsystem.ClawState.CLOSED);



            robot.intake.loop();
            robot.lift.loop();

            robot.write();

            for(LynxModule module : robot.getControllers()){
                module.clearBulkCache();
            }

        }


        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, testing),
                        new LiftPositionCommand(robot.lift, 26, 2),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT))
                        ),

                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DUNK)),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toConeStackAfterPreload),
                        new SequentialCommandGroup(
                                new WaitCommand(50),
                                new LiftPositionCommand(robot.lift, 5.5, 2)
                        ),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                ),
                new WaitCommand(250),
                new AutoCycleCommandV5(robot, toPoleAfterConeOnePickUp, toConeTwoAfterConeOneDeposit, 4.8),
                new WaitCommand(250),
                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 4),
                new WaitCommand(250),
                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 3.5),
                new WaitCommand(250),
                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 3.4),
                new WaitCommand(250),
                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 3)

                ));


        robot.reset();

        while(opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();



            robot.intake.loop();
            robot.lift.loop();


            robot.write();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("Lift Pos", robot.lift.getLiftPos());
            telemetry.addData("Lift Target Pos", robot.lift.liftTargetPosition);

            telemetry.addData("Robot Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());
            loopTime = loop;
            telemetry.update();

            for(LynxModule module : robot.getControllers()){
                module.clearBulkCache();
            }




        }
    }




}

