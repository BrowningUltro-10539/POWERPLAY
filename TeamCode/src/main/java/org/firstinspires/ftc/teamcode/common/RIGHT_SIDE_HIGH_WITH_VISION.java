package org.firstinspires.ftc.teamcode.common;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV6;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV7Medium;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV7MediumWithPark;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoPreloadCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoPreloadCommandV2Medium;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.rr.util.AutoConstants;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RIGHT_SIDE_HIGH", group = "COMPETITION")
public class RIGHT_SIDE_HIGH_WITH_VISION extends LinearOpMode {

    private Robot robot;
    private ElapsedTime timer;

    private double loopTime = 0;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 16;
    int MIDDLE = 14;
    int RIGHT = 19;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, true);

        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(270));
        robot.driveSubsystem.setPoseEstimate(startPose);

        robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
        robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
        sleep(1500);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);


        TrajectorySequence toPolePreload = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36.2, -4.5))
                .turn(Math.toRadians(60))
                .back(1.75)
                .build();

        Trajectory toConeStackAfterPreload = robot.driveSubsystem.trajectoryBuilder(toPolePreload.end())
                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X, AutoConstants.CONE_STACK_LINE_Y))
                .build();

        TrajectorySequence toPoleAfterConeIntakeCycle1 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X, AutoConstants.POLE_SPLINE_Y), Math.toRadians(AutoConstants.POLE_HEADING - 1))
                .build();

        TrajectorySequence toConeStackAfterPoleDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle1.end())
                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X, AutoConstants.CONE_STACK_LINE_Y))
                .build();

        TrajectorySequence toPoleAfterConeIntake = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPoleDeposit.end())
                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X, AutoConstants.POLE_SPLINE_Y), Math.toRadians(AutoConstants.POLE_HEADING - 1))
                .build();

        TrajectorySequence toConeStackAfterPoleDepositCycle2 = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle1.end())
                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X + 1, AutoConstants.CONE_STACK_LINE_Y))
                .build();

        TrajectorySequence toPoleAfterConeIntakeCycle2 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPoleDepositCycle2.end())
                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X + 1.5, AutoConstants.POLE_SPLINE_Y + 1.5), Math.toRadians(AutoConstants.POLE_HEADING - 1))
                .build();

        TrajectorySequence toConeStackAfterPoleDepositCycle3 = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle2.end())
                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X + 3, AutoConstants.CONE_STACK_LINE_Y))
                .build();

        TrajectorySequence toPoleAfterConeIntakeCycle3 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPoleDepositCycle3.end())
                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X + 4, AutoConstants.POLE_SPLINE_Y + 2.5), Math.toRadians(AutoConstants.POLE_HEADING - 3))
                .build();

        TrajectorySequence toParkingSpotOne = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
                .forward(5)
                .lineToLinearHeading(new Pose2d(20, -12, Math.toRadians(270)))
                .build();

        TrajectorySequence toParkingSpotTwo = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
                .forward(3)
                .lineToLinearHeading(new Pose2d(45, -12, Math.toRadians(0)))
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence toParkingSpotThree = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
                .forward(5)
                .lineToLinearHeading(new Pose2d(67, -10, Math.toRadians(270)))

                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            robot.reset();

            for (LynxModule module : robot.getControllers()) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }


            robot.intake.loop();
            robot.lift.loop();

            robot.write();

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }



        if (tagOfInterest == null) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterPoleDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterPoleDepositCycle3, 2.8),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle3, toParkingSpotTwo, 0)
                    )
            );
        } else if (tagOfInterest.id == 16) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterPoleDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterPoleDepositCycle3, 2.8),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle3, toParkingSpotOne, 0)
                    )
            );

        } else if (tagOfInterest.id == 14) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterPoleDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterPoleDepositCycle3, 2.8),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle3, toParkingSpotTwo, 0)
                    )
            );

        } else if (tagOfInterest.id == 19) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterPoleDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterPoleDepositCycle3, 2.8),
                            new WaitCommand(25),
                            new AutoCycleCommandV6(robot, toPoleAfterConeIntakeCycle3, toParkingSpotThree, 0)
                    )
            );

        }

        robot.reset();

        while (opModeIsActive()) {
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

            for (LynxModule module : robot.getControllers()) {
                module.clearBulkCache();
            }

        }


    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}