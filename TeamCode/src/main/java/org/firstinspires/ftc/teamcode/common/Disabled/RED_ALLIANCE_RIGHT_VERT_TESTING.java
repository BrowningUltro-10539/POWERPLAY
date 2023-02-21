package org.firstinspires.ftc.teamcode.common.Disabled;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Old.AutoCycleCommandV5;
import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.ParkingState;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Config
@Disabled
public class RED_ALLIANCE_RIGHT_VERT_TESTING extends LinearOpMode {
    private Robot robot;
    private ElapsedTime timer;
    private double loopTime = 0;

    private ParkingState parkingState = ParkingState.ONE;

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(0.166, 0,0,0,0);

    public TrajectorySequence parkingTraj;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, true);





        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(270));
        robot.driveSubsystem.setPoseEstimate(startPose);

        robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
        robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
        sleep(2000);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });




        //Make a separate trajectory sequence for the spline portion where you get to turn and then raise slides at that point.


        TrajectorySequence testing = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                //Dropping off preload to pole
                //Dropping off preload to pole
                .lineTo(new Vector2d(36, -23))
                .splineToSplineHeading(new Pose2d(35, -8, Math.toRadians(325)), Math.toRadians(120))
                .back(1)
                .build();

        TrajectorySequence toConeStackAfterPreload = robot.driveSubsystem.trajectorySequenceBuilder(testing.end())
                .splineTo(new Vector2d(64, -12.5), Math.toRadians(2))
//                .lineTo(new Vector2d(64, -8))
                .build();

        TrajectorySequence toPoleAfterConeOnePickUp = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
                .setReversed(true)
                //SCORE CONE ONE
                .lineTo(new Vector2d(47, -11))
                .splineToSplineHeading(new Pose2d(35.5, -6.5, Math.toRadians(325)), Math.toRadians(120))


                .build();

        TrajectorySequence toConeTwoAfterConeOneDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeOnePickUp.end())
                .setReversed(false)
                //GO TO CONE TWO
                .splineTo(new Vector2d(68, -13), Math.toRadians(0))
//                .lineTo(new Vector2d(65, -8))
                .build();



        TrajectorySequence toPoleAfterConeTwoPickUp = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
                .setReversed(true)
                //SCORE CONE ONE
                .lineTo(new Vector2d(47, -11))
                .splineToSplineHeading(new Pose2d(40.5, -5, Math.toRadians(325)), Math.toRadians(120))

                .build();

        TrajectorySequence toConeThreeAfterConeTwoDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeTwoPickUp.end())
                .setReversed(false)
                //GO TO CONE TWO
                .splineTo(new Vector2d(69, -13), Math.toRadians(0))
//                .lineTo(new Vector2d(65, -8))
                .build();

        TrajectorySequence toPoleAfterConeThreePickUp = robot.driveSubsystem.trajectorySequenceBuilder(toConeThreeAfterConeTwoDeposit.end())
                .setReversed(true)
                //SCORE CONE ONE
                .lineTo(new Vector2d(47, -11))
                .splineToSplineHeading(new Pose2d(42, -5, Math.toRadians(325)), Math.toRadians(120))
                .build();

        TrajectorySequence toConeFourAfterConeThreeDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeThreePickUp.end())
                .setReversed(false)
                //GO TO CONE TWO
                .splineTo(new Vector2d(67, -11.25), Math.toRadians(0))
//                .lineTo(new Vector2d(65, -8))
                .build();


        TrajectorySequence toParkingSpaceThree = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeThreePickUp.end())
                .splineTo(new Vector2d(68, -12), Math.toRadians(0))
                .turn(90)
                .build();

        TrajectorySequence toParkingSpaceTwo = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeThreePickUp.end())
                .splineTo(new Vector2d(45, -12), Math.toRadians(0))
                .turn(90)
                .build();

        TrajectorySequence toParkingSpaceOne = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeThreePickUp.end())
                .strafeRight(5)
                .lineToLinearHeading(new Pose2d(20, -12, Math.toRadians(90)))
                .build();



        while(!isStarted() && !isStopRequested()) {
            robot.read();

            for (LynxModule module : robot.getControllers()) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            robot.intake.loop();
            robot.lift.loop();

            robot.write();

            if (currentDetections.size() != 0) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 16) {
                        parkingState = ParkingState.ONE;
                        telemetry.addLine("ONE");
                    } else if (detection.id == 14) {
                        parkingState = ParkingState.TWO;
                        telemetry.addLine("TWO");
                    } else if (detection.id == 19) {
                        parkingState = ParkingState.THREE;
                        telemetry.addLine("THREE");
                    }
                    telemetry.update();
                }
            }

            telemetry.update();
        }

        if(parkingState == ParkingState.ONE){
            parkingTraj = toParkingSpaceOne;
        } else if(parkingState == ParkingState.TWO){
            parkingTraj = toParkingSpaceTwo;
        } else {
            parkingTraj = toParkingSpaceThree;
        }

        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, testing),
                        new SequentialCommandGroup(
                                new WaitCommand(1450),
//                                new LiftPositionCommand(robot.lift, 5, 2)
                                new LiftPositionCommand(robot.lift, 21, 2)
                        )
                        ),

                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                new LiftPositionCommand(robot.lift, 24, 2),
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
                                new LiftPositionCommand(robot.lift, 5.5, 2)
                        ),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                ),

                new AutoCycleCommandV5(robot, toPoleAfterConeOnePickUp, toConeTwoAfterConeOneDeposit, 4.75),
                new WaitCommand(25),
                new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 4.25),
                new WaitCommand(25),
                new AutoCycleCommandV5(robot, toPoleAfterConeThreePickUp, parkingTraj, 0)
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

