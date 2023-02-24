package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoPreloadCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoConstants;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.teamcode.util.AutoConstants.*;

@Autonomous(name = "RIGHT_OFFICIAL_AUTO", group = "COMPETITION")
public class RightAutoWithNavX extends LinearOpMode {
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


        TrajectorySequence toPolePreload = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(AutoConstants.PRELOAD_LINE_X, AutoConstants.PRELOAD_LINE_Y))
                .splineTo(new Vector2d(AutoConstants.PRELOAD_SPLINE_POLE_X, AutoConstants.PRELOAD_SPLINE_POLE_Y), AutoConstants.PRELOAD_POLE_HEADING)
                .build();

        TrajectorySequence toConeStackAfterPreload = robot.driveSubsystem.trajectorySequenceBuilder(toPolePreload.end())
                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X, AutoConstants.CONE_STACK_LINE_Y))
                .build();

        TrajectorySequence toPoleAfterConeIntake = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X, AutoConstants.POLE_SPLINE_Y),AutoConstants.POLE_HEADING)
                .build();

        TrajectorySequence toConeStackAfterPoleDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntake.end())
                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X, AutoConstants.CONE_STACK_LINE_Y))
                .build();

        robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
        robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
        sleep(1000);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);


//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.enable();


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

        while(!isStarted()){
            robot.read();

//            telemetry.addLine(pipeline.getPosition().toString());
//            telemetry.update();
//            PhotonCore.CONTROL_HUB.clearBulkCache();
//            PhotonCore.EXPANSION_HUB.clearBulkCache();
            robot.write();
        }

//        SleeveDetection.ParkingPosition position = pipeline.getPosition();

        waitForStart();
//        camera.stopStreaming();
        //robot.startIMUThread(this);


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
//                        new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload)
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPolePreload)
                )

        );

        robot.reset();

        while(opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.intake.loop();
            robot.lift.loop();

            telemetry.addData("Current Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            robot.write();
//            PhotonCore.CONTROL_HUB.clearBulkCache();

        }






    }
}
