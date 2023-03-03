package org.firstinspires.ftc.teamcode.common;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV7Medium;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV7MediumWithPark;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoPreloadCommandV2Medium;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.rr.util.AutoConstants;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RIGHT_SIDE_MEDIUM")
public class RIGHT_SIDE_MEDIUM_WITH_VISION extends LinearOpMode {

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


        TrajectorySequence toPolePreloadMedium = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36, -35))
                .splineTo(new Vector2d(32, -28.5), Math.toRadians(140))

                .build();

        TrajectorySequence toConeStack = robot.driveSubsystem.trajectorySequenceBuilder(toPolePreloadMedium.end())
                .lineToLinearHeading(new Pose2d(35, -9, Math.toRadians(0)))
                .lineTo(new Vector2d(63, -9))
                .build();

        TrajectorySequence toPoleAfterConeIntakeCycle1 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStack.end())
                .lineTo(new Vector2d(48, -12.25))
                .setTurnConstraint(Math.toRadians(75), Math.toRadians(75))
                .splineTo(new Vector2d(34, -18.3), Math.toRadians(207))
                .build();

        TrajectorySequence toConeStackAfterConeDepositCycle2 = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle1.end())
                .splineTo(new Vector2d(48, -11), Math.toRadians(0))
                .lineTo(new Vector2d(62.9, -11 + 2.35))
                .build();


        TrajectorySequence toPoleAfterConeIntakeCycle2 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterConeDepositCycle2.end())
                .lineTo(new Vector2d(48, -12.25))
                .setTurnConstraint(Math.toRadians(75), Math.toRadians(75))
                .splineTo(new Vector2d(36, -20), Math.toRadians(207))
                .build();


        TrajectorySequence toConeStackAfterConeDepositCycle3 = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle2.end())
                .splineTo(new Vector2d(48, -8.25), Math.toRadians(0))
                .lineTo(new Vector2d(64.7, -11 + 2.5))
                .build();

        TrajectorySequence toPoleAfterConeIntakeCycle3 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterConeDepositCycle3.end())
                .lineTo(new Vector2d(48, -12.25))
                .setTurnConstraint(Math.toRadians(75), Math.toRadians(75))
                .splineTo(new Vector2d(36.4 + 0.5, -18 - 0.5), Math.toRadians(207))
                .build();

        TrajectorySequence toParkingSpotOne = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
                .lineTo(new Vector2d(40, -12))
                .lineToLinearHeading(new Pose2d(15, -12, Math.toRadians(270)))
                .build();

        TrajectorySequence toParkingSpotTwo = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
                .forward(3)
                .lineToLinearHeading(new Pose2d(44, -12, Math.toRadians(270)))
                .build();

        TrajectorySequence toParkingSpotThree = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
                .forward(2)
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
                            new AutoPreloadCommandV2Medium(robot, toPolePreloadMedium, toConeStack),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterConeDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterConeDepositCycle3, AutoConstants.SLIDE_HEIGHTS[2]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7MediumWithPark(robot, toPoleAfterConeIntakeCycle3, toParkingSpotTwo, 0)

                    )
            );
        } else if (tagOfInterest.id == 16) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new AutoPreloadCommandV2Medium(robot, toPolePreloadMedium, toConeStack),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterConeDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterConeDepositCycle3, AutoConstants.SLIDE_HEIGHTS[2]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7MediumWithPark(robot, toPoleAfterConeIntakeCycle3, toParkingSpotOne, 0)

                    )
            );

        } else if (tagOfInterest.id == 14) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new AutoPreloadCommandV2Medium(robot, toPolePreloadMedium, toConeStack),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterConeDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterConeDepositCycle3, AutoConstants.SLIDE_HEIGHTS[2]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7MediumWithPark(robot, toPoleAfterConeIntakeCycle3, toParkingSpotTwo, 0)

                    )
            );

        } else if (tagOfInterest.id == 19) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new AutoPreloadCommandV2Medium(robot, toPolePreloadMedium, toConeStack),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterConeDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterConeDepositCycle3, AutoConstants.SLIDE_HEIGHTS[2]),
                            new WaitCommand(10),
                            new AutoCycleCommandV7MediumWithPark(robot, toPoleAfterConeIntakeCycle3, toParkingSpotThree, 0)

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