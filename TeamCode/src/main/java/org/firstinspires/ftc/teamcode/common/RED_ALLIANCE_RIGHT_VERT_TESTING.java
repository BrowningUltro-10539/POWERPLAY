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

        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(270));        robot.driveSubsystem.setPoseEstimate(startPose);

        TrajectorySequence testing = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                //Dropping off preload to pole
                //Dropping off preload to pole
                .lineTo(new Vector2d(36, -23))
                .splineToSplineHeading(new Pose2d(35, -7, Math.toRadians(325)), Math.toRadians(120))
                .back(1)
                .build();

        TrajectorySequence toConeStackAfterPreload = robot.driveSubsystem.trajectorySequenceBuilder(testing.end())
                .splineTo(new Vector2d(47, -11), Math.toRadians(2))
                .lineTo(new Vector2d(64, -7.75))
                .build();

        TrajectorySequence toPoleAfterConeOnePickUp = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
                .setReversed(true)
                //SCORE CONE ONE
                .lineTo(new Vector2d(47, -11))
                .splineToSplineHeading(new Pose2d(36, -7, Math.toRadians(315)), Math.toRadians(145))
                .back(1)
                .build();

        TrajectorySequence toConeTwoAfterConeOneDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeOnePickUp.end())
                .setReversed(false)
                //GO TO CONE TWO
                .splineTo(new Vector2d(47, -11), Math.toRadians(0))
                .lineTo(new Vector2d(65, -7.75))
                .build();



        TrajectorySequence toPoleAfterConeTwoPickUp = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
                .setReversed(true)
                //SCORE CONE ONE
                .lineTo(new Vector2d(47, -11))
                .splineToSplineHeading(new Pose2d(38, -5, Math.toRadians(315)), Math.toRadians(145))


                .build();

        TrajectorySequence toConeThreeAfterConeTwoDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeTwoPickUp.end())
                .setReversed(false)t
                //GO TO CONE TWO
                .splineTo(new Vector2d(47, -11), Math.toRadians(0))
                .lineTo(new Vector2d(65, -7.75))
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
                        new LiftPositionCommand(robot.lift, 5, 2)
                        ),

                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                new LiftPositionCommand(robot.lift, 24, 2),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new LiftPositionCommand(robot.lift, 20, 2)
                ),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toConeStackAfterPreload),
                        new SequentialCommandGroup(
                                new WaitCommand(50),
                                new LiftPositionCommand(robot.lift, 4.5, 2),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                        )

                ),
                new WaitCommand(100),
                new AutoCycleCommandV5(robot, toPoleAfterConeOnePickUp, toConeTwoAfterConeOneDeposit, 3.75),
                new WaitCommand(100),
                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 3),
                new WaitCommand(100),
                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 2.5)
//                new WaitCommand(250),
//                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 3.5),
//                new WaitCommand(250),
//                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 3.4),
//                new WaitCommand(250),
//                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 3)

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

