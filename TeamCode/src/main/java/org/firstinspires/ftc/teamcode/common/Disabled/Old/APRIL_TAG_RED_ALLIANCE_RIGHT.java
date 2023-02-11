//package org.firstinspires.ftc.teamcode.common.Disabled;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
//import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Autonomous
//@Disabled
//public class APRIL_TAG_RED_ALLIANCE_RIGHT extends LinearOpMode {
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    int ID_TAG_OF_INTEREST_1 = 16;
//    int ID_TAG_OF_INTEREST_2 = 14;
//    int ID_TAG_OF_INTEREST_3 = 19;
//    // Tag ID 18 from the 36h11 family
//
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//    AprilTagDetection tagOfInterest = null;
//
//    @Override
//    public void runOpMode()
//    {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//
//        CommandScheduler.getInstance().reset();
//        Robot robot = new Robot(hardwareMap, true);
//
//        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));
//        robot.driveSubsystem.setPoseEstimate(startPose);
//
//        Trajectory traj = robot.driveSubsystem.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(35, -32))
//                .build();
//
//        Trajectory zone1Traj = robot.driveSubsystem.trajectoryBuilder(traj.end())
//                .lineTo(new Vector2d(10, -32))
//                .build();
//
//        Trajectory zone2Traj = robot.driveSubsystem.trajectoryBuilder(traj.end())
//                .lineTo(new Vector2d(34, -32))
//                .build();
//
//        Trajectory zone3Traj = robot.driveSubsystem.trajectoryBuilder(traj.end())
//                .lineTo(new Vector2d(60, -32))
//                .build();
//
//
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        while (!isStarted() && !isStopRequested())
//        {
//
//            robot.read();
//
//            for(LynxModule module : robot.getControllers()){
//                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            }
//
//            robot.intake.setArmTargetAngle(40);
//
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == ID_TAG_OF_INTEREST_1)
//                    {
//                        tagOfInterest = tag;
//
//                        tagFound = true;
//                        break;
//                    } else if(tag.id == ID_TAG_OF_INTEREST_2){
//                        tagOfInterest = tag;
//
//                        tagFound = true;
//                        break;
//                    } else if(tag.id == ID_TAG_OF_INTEREST_3) {
//
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            robot.intake.loop();
//            robot.lift.loop();
//
//            robot.write();
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        robot.reset();
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//
//        /* Actually do something useful */
//        if(tagOfInterest == null)
//        {
//            /*
//             * Insert your autonomous code here, presumably running some default configuration
//             * since the tag was never sighted during INIT
//             */
//        }
//        else
//        {
//            /*
//             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
//             */
//
//            // e.g.
//            if(tagOfInterest.id == ID_TAG_OF_INTEREST_1)
//            {
//                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                        new TrajectoryFollowerCommand(robot.driveSubsystem, traj),
//                        //new InstantCommand(() -> robot.lift.setTargetTurretAngle(27))
//
//                        // new WaitCommand(1000),
//                        //   new LiftAndDropConeCommand(robot),
//                        new WaitCommand(2000),
//                        new TrajectoryFollowerCommand(robot.driveSubsystem, zone1Traj)
//                        // new InstantCommand(() -> robot.intake.setArmTargetAngle(90))
//                ));
//            }
//            else if(tagOfInterest.id == ID_TAG_OF_INTEREST_2)
//            {
//                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                        new TrajectoryFollowerCommand(robot.driveSubsystem, traj),
//                        //new InstantCommand(() -> robot.lift.setTargetTurretAngle(27))
//
//                        // new WaitCommand(1000),
//                        //   new LiftAndDropConeCommand(robot),
//                        new WaitCommand(2000),
//                        new TrajectoryFollowerCommand(robot.driveSubsystem, zone2Traj)
//                        // new InstantCommand(() -> robot.intake.setArmTargetAngle(90))
//                ));
//            }
//            else if(tagOfInterest.id == ID_TAG_OF_INTEREST_3)
//            {
//                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                        new TrajectoryFollowerCommand(robot.driveSubsystem, traj),
//                        //new InstantCommand(() -> robot.lift.setTargetTurretAngle(27))
//
//                        // new WaitCommand(1000),
//                        //   new LiftAndDropConeCommand(robot),
//                        new WaitCommand(2000),
//                        new TrajectoryFollowerCommand(robot.driveSubsystem, zone3Traj)
//                        // new InstantCommand(() -> robot.intake.setArmTargetAngle(90))
//                ));
//            }
//        }
//
//
//        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {
//            robot.read();
//
//            CommandScheduler.getInstance().run();
//
//            robot.intake.setLiftTurretState(robot.lift.turretState);
//            robot.intake.setLiftTurretCurrentAngle(robot.lift.getTurretAngle());
//
//            robot.intake.loop();
//            robot.lift.loop();
//
//
//            robot.write();
//
//            for(LynxModule module : robot.getControllers()){
//                module.clearBulkCache();
//            }
//        }
//    }
//
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//}
