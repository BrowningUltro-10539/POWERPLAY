package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV6;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "RIGHT_OLD_AUTO", group = "COMPETITION")
@Disabled

public class RightAuto extends LinearOpMode {
    private Robot robot;
    private OpenCvCamera camera;
    private SleeveDetection pipeline = new SleeveDetection();

    private double loopTime;

    private Pose2d startPose = new Pose2d(33, -62, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, true);


        robot.driveSubsystem.setPoseEstimate(startPose);

        Trajectory toPolePreload = robot.driveSubsystem.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(36, -16, Math.toRadians(270)))
                .splineTo(new Vector2d(35.9, -5), Math.toRadians(150))
                .build();

        Trajectory toPolePreloadV2 = robot.driveSubsystem.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(33, -12))
                .splineTo(new Vector2d(29, -5), Math.toRadians(138))
                .build();

        Trajectory toConeStackAfterPreload = robot.driveSubsystem.trajectoryBuilder(toPolePreload.end())
                .splineTo(new Vector2d(45, -13), Math.toRadians(0))
                .lineTo(new Vector2d(63, -13))
                .build();

        Trajectory toConeStackAfterPreloadV2 = robot.driveSubsystem.trajectoryBuilder(toPolePreload.end())
                .splineTo(new Vector2d(48, -12.5), Math.toRadians(0))
                .lineTo(new Vector2d(62.5, -12.5))
                .build();

        Trajectory toPoleAfterConeStack = robot.driveSubsystem.trajectoryBuilder(toConeStackAfterPreload.end())
                .lineToSplineHeading(new Pose2d(45, -12, Math.toRadians(0)))
                .splineTo(new Vector2d(35.9, -5), Math.toRadians(150))
                .build();

        Trajectory toPoleAfterConeStackV2 = robot.driveSubsystem.trajectoryBuilder(toConeStackAfterPreload.end())
                .lineTo(new Vector2d(48, -12.5))
                .splineTo(new Vector2d(29, -5), Math.toRadians(138))
                .build();

        Trajectory toConeStackAfterPole = robot.driveSubsystem.trajectoryBuilder(toPoleAfterConeStack.end())
                .splineTo(new Vector2d(45, -13), Math.toRadians(0))
                .lineTo(new Vector2d(63, -13))
                .build();



//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        camera.setPipeline(pipeline);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() { camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
//
//            @Override
//            public void onError(int errorCode) { }
//        });

        while(isStarted()){
            robot.read();

//            telemetry.addLine(pipeline.getPosition().toString());
//            telemetry.update();


            robot.write();
        }

//        SleeveDetection.ParkingPosition position = pipeline.getPosition();
        robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
        robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
        sleep(1000);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        waitForStart();
//        camera.stopStreaming();
        //robot.startIMUThread(this);


        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new TrajectoryFollowerCommand(robot.driveSubsystem, toPolePreloadV2)),
                        new SequentialCommandGroup(
                                new WaitCommand(700),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                                        new NewLiftPositionCommand(robot.lift, 20, 200, 200, 2)
                                ),
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.POLE_DUNK)),
                                        new NewLiftPositionCommand(robot.lift, 18.5, 200, 200, 2)
                                ),
                                new WaitCommand(250),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))

                                    )
                            ),
                            new WaitCommand(1000)
//                            new TrajectoryFollowerCommand(robot.driveSubsystem, toConeStackAfterPreloadV2)
//                new WaitCommand(1000),
//                new ParallelCommandGroup(
//                        new TrajectoryFollowerCommand(robot.driveSubsystem, toConeStackAfterPreload),
//                        new SequentialCommandGroup(
//                                new WaitCommand(100),
//                                new NewLiftPositionCommand(robot.lift, 3.75, 200, 200, 2)
//                        ),
//                        new SequentialCommandGroup(
//                                new WaitCommand(200),
//                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
//                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
//                        )
//
//                ),
//                new WaitCommand(100),
//                new AutoCycleCommandV6(robot, toPoleAfterConeStack, toConeStackAfterPole, 3.25)
        )


        );

        robot.reset();

        while(opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.intake.loop();
            robot.lift.loop();

//            telemetry.addData("Lift Position", robot.lift.getLiftPos());
            telemetry.addData("Current Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();

            robot.write();

        }






    }
}
