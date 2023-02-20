//package org.firstinspires.ftc.teamcode.common.Disabled;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.NewAutoCommands.LiftAndDropConeCommand;
//import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
//
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Autonomous
//@Disabled
//public class BLUE_ALLIANCE_RIGHT extends LinearOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//    int ZONE_1 = 16;
//    int ZONE_2 = 14;
//    int ZONE_3 = 19;
//
//    boolean zone1;
//    boolean zone2;
//    boolean zone3;
//
//
//    private OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        CommandScheduler.getInstance().reset();
//        Robot robot = new Robot(hardwareMap, true);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.04318,0,0,0,0);
//        camera.setPipeline(aprilTagDetectionPipeline);
//
//        Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(270));
//        robot.driveSubsystem.setPoseEstimate(startPose);
//
//        Trajectory traj = robot.driveSubsystem.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(-35, 23))
//                .splineTo(new Vector2d(-52, 12), Math.toRadians(180))
//                .build();
//
//        Trajectory zone1Traj = robot.driveSubsystem.trajectoryBuilder(traj.end())
//                        .lineTo(new Vector2d(-12, 12))
//                                .build();
//
//        Trajectory zone2Traj = robot.driveSubsystem.trajectoryBuilder(traj.end())
//                .lineTo(new Vector2d(-35, 12))
//                .build();
//
//        Trajectory zone3Traj = robot.driveSubsystem.trajectoryBuilder(traj.end())
//                .lineTo(new Vector2d(-64, 12))
//                .build();
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() { camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
//
//            @Override
//            public void onError(int errorCode) {}
//        });
//
//
//        while(!isStarted()) {
//            robot.read();
//
//            for(LynxModule module : robot.getControllers()){
//                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            }
//
//            robot.intake.setArmTargetAngle(50);
//
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() > 0){
//                for(AprilTagDetection detection: currentDetections){
//                    if(detection.id == ZONE_1){
//                        zone1 = true;
//                        zone2 = false;
//                        zone3 = false;
//                    } else if(detection.id == ZONE_2){
//                        zone2 = true;
//                        zone1 = false;
//                        zone3 = false;
//                    } else if(detection.id == ZONE_3){
//                        zone3 = true;
//                        zone2 = false;
//                        zone1 = false;
//                    }
//                }
//            }
//            //16, 14, 19
//
//
//            robot.intake.loop();
//            robot.lift.loop();
//
//            robot.write();
//        }
//
//        telemetry.addData("ZONE 1: ", zone1);
//        telemetry.addData("ZONE 2: ", zone2);
//        telemetry.addData("ZONE 3: ", zone3);
//        telemetry.update();
//
//
//
//        waitForStart();
//
//
//        if(zone1){
//
//            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                    new ParallelCommandGroup(
//                            new TrajectoryFollowerCommand(robot.driveSubsystem, traj),
//                            new InstantCommand(() -> robot.lift.setTargetTurretAngle(26))
//                    ),
//                    new WaitCommand(1000),
//                    new LiftAndDropConeCommand(robot),
//                    new TrajectoryFollowerCommand(robot.driveSubsystem, zone1Traj)
//            ));
//        }
//
//        if(zone2){
//            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                    new ParallelCommandGroup(
//                            new TrajectoryFollowerCommand(robot.driveSubsystem, traj),
//                            new InstantCommand(() -> robot.lift.setTargetTurretAngle(26))
//                            ),
//                    new WaitCommand(1000),
//                    new LiftAndDropConeCommand(robot),
//                    new TrajectoryFollowerCommand(robot.driveSubsystem, zone2Traj)
//            ));
//        }
//
//        if(zone3){
//            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                    new ParallelCommandGroup(
//                            new TrajectoryFollowerCommand(robot.driveSubsystem, traj),
//                            new InstantCommand(() -> robot.lift.setTargetTurretAngle(26))
//                            ),
//                    new WaitCommand(1000),
//                    new LiftAndDropConeCommand(robot),
//                    new TrajectoryFollowerCommand(robot.driveSubsystem, zone3Traj)
//            ));
//        }
//
//
//
//        robot.reset();
//
//        while(opModeIsActive()){
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
//            double loop = System.nanoTime();
//            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            telemetry.addData("Lift Pos", robot.lift.getLiftPos());
//            telemetry.addData("Lift Target Pos", robot.lift.liftTargetPosition);
//            telemetry.addData("Lift Turret Power", robot.lift.turretPower);
//            loopTime = loop;
//            telemetry.update();
//
//            for(LynxModule module : robot.getControllers()){
//                module.clearBulkCache();
//            }
//
//
//
//
//        }
//    }
//
//
//
//
//}