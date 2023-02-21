package org.firstinspires.ftc.teamcode.common.Disabled;/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


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
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled
public class LEFT_LEFT_APRIL_TAG_RED_ALLIANCE_LEFT_VERT_TESTING extends LinearOpMode {

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

        robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
        robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
        sleep(2000);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);

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


        //HARDWARE MAPPING HERE etc.
        TrajectorySequence testing = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                //Dropping off preload to pole
                //Dropping off preload to pole
                .lineTo(new Vector2d(36, -23))
                .splineToSplineHeading(new Pose2d(35, -8, Math.toRadians(215)), Math.toRadians(120))
                .back(1)
                .build();

        TrajectorySequence toConeStackAfterPreload = robot.driveSubsystem.trajectorySequenceBuilder(testing.end())
                .splineTo(new Vector2d(65, -13), Math.toRadians(2))
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
                .splineTo(new Vector2d(67, -13), Math.toRadians(0))
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
                .splineTo(new Vector2d(67, -12), Math.toRadians(0))
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


        TrajectorySequence toParkingSpaceOne = robot.driveSubsystem.trajectorySequenceBuilder(testing.end())
                .lineToLinearHeading(new Pose2d(10, -13, Math.toRadians(-90)))
                .forward(4)
                .build();

        TrajectorySequence toParkingSpaceTwo = robot.driveSubsystem.trajectorySequenceBuilder(testing.end())
                .lineToLinearHeading(new Pose2d(40, -13, Math.toRadians(-90)))
                .forward(4)
                .build();

        TrajectorySequence toParkingSpaceThree = robot.driveSubsystem.trajectorySequenceBuilder(testing.end())
                .lineToLinearHeading(new Pose2d(62, -13, Math.toRadians(-90)))
                .forward(4)
                .build();







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

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)


        if (tagOfInterest == null) {
            //default path
        } else if (tagOfInterest.id == 16) {
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, testing),
                            new SequentialCommandGroup(
                                    new WaitCommand(1450),
//                                new LiftPositionCommand(robot.lift, 5, 2)
                                    new NewLiftPositionCommand(robot.lift, 22, 50, 60, 2)
                            )
                    ),

                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DUNK)),
//                new LiftPositionCommand(robot.lift, 24, 2),
                    new WaitCommand(250),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                            new NewLiftPositionCommand(robot.lift, 21, 50, 60, 2)
                    ),
                    new WaitCommand(250),
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                    new WaitCommand(250),
                    new ParallelCommandGroup(
                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toParkingSpaceOne),
                            new SequentialCommandGroup(
                                    new WaitCommand(50),
                                    new NewLiftPositionCommand(robot.lift, 0, 50, 60, 2),
                                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                            )

                    )

//                            new WaitCommand(25),
//                            new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 4.25),
//                            new WaitCommand(25),
//                            new AutoCycleCommandV5(robot, toPoleAfterConeThreePickUp, toParkingSpaceOne, 0)
            ));

        } else if (tagOfInterest.id == 14) {
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, testing),
                            new SequentialCommandGroup(
                                    new WaitCommand(1450),
//                                new LiftPositionCommand(robot.lift, 5, 2)
                                    new NewLiftPositionCommand(robot.lift, 22, 50, 60, 2)
                            )
                    ),

                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DUNK)),
//                new LiftPositionCommand(robot.lift, 24, 2),
                    new WaitCommand(250),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                            new NewLiftPositionCommand(robot.lift, 21, 50, 60, 2)
                    ),
                    new WaitCommand(250),
//                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toParkingSpaceTwo),

                    new ParallelCommandGroup(
                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toParkingSpaceTwo),
                            new SequentialCommandGroup(
                                    new WaitCommand(50),
                                    new NewLiftPositionCommand(robot.lift, 0, 50, 60, 2),
                                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                            )

                    )

//                            new AutoCycleCommandV5(robot, toPoleAfterConeOnePickUp, toConeTwoAfterConeOneDeposit, 4.75),
//                            new WaitCommand(25),
//                            new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 4.25),
//                            new WaitCommand(25),
//                            new AutoCycleCommandV5(robot, toPoleAfterConeThreePickUp, toParkingSpaceTwo, 0)
            ));
        } else if (tagOfInterest.id == 19) {
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, testing),
                            new SequentialCommandGroup(
                                    new WaitCommand(1450),
//                                new LiftPositionCommand(robot.lift, 5, 2)
                                    new NewLiftPositionCommand(robot.lift, 22, 50, 60, 2)
                            )
                    ),

                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DUNK)),
//                new LiftPositionCommand(robot.lift, 24, 2),
                    new WaitCommand(250),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                            new NewLiftPositionCommand(robot.lift, 21, 50, 60, 2)
                    ),
                    new WaitCommand(250),
                    new ParallelCommandGroup(
                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toParkingSpaceThree),
                            new SequentialCommandGroup(
                                    new WaitCommand(50),
                                    new NewLiftPositionCommand(robot.lift, 0, 50, 60, 2),
                                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                            )
                    )));
        }


//                           ),
//
//                            new AutoCycleCommandV5(robot, toPoleAfterConeOnePickUp, toConeTwoAfterConeOneDeposit, 4.75),
//                            new WaitCommand(25),
//                            new AutoCycleCommandV5(robot, toPoleAfterConeTwoPickUp, toConeThreeAfterConeTwoDeposit, 4.25),
//                            new WaitCommand(25),
//                            new AutoCycleCommandV5(robot, toPoleAfterConeThreePickUp, toParkingSpaceThree, 0)
//                    ));

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