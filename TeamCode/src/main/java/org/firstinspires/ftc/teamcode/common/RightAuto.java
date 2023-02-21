package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "RIGHT_OFFICIAL_AUTO", group = "COMPETITION")
@Disabled
public class RightAuto extends LinearOpMode {
    private Robot robot;
    private OpenCvCamera camera;
    private SleeveDetection pipeline = new SleeveDetection();

    private double loopTime;

    private Pose2d startPose = new Pose2d(36, -62, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, true);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();

        robot.driveSubsystem.setPoseEstimate(startPose);

        TrajectorySequence toPolePreload = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(36, -20, Math.toRadians(270)))
                .splineTo(new Vector2d(31, -5), Math.toRadians(145))
                .build();

        TrajectorySequence toConeStackAfterPreload = robot.driveSubsystem.trajectorySequenceBuilder(toPolePreload.end())
                .splineTo(new Vector2d(45, -11.5), Math.toRadians(0))
                .lineTo(new Vector2d(62, -11.5))
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

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();

            robot.write();
        }

//        SleeveDetection.ParkingPosition position = pipeline.getPosition();
        waitForStart();
        camera.stopStreaming();
        //robot.startIMUThread(this);


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPolePreload),
                                new SequentialCommandGroup(
                                        new WaitCommand(1200),
                                        new NewLiftPositionCommand(robot.lift, 21.5, 200, 200, 2)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DUNK))
                ),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                new WaitCommand(50),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new NewLiftPositionCommand(robot.lift, 19, 200, 200, 2)
                )
        )
        );

        robot.reset();

        while(opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.intake.loop();
            robot.lift.loop();


            telemetry.addData("Lift Position", robot.lift.getLiftPos());
            telemetry.addData("Current Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();

            robot.write();
            PhotonCore.CONTROL_HUB.clearBulkCache();
        }






    }
}
