package org.firstinspires.ftc.teamcode.common.Disabled;
        /*
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
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV6;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoPreloadCommand;
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

@Autonomous
@Disabled
public class APRIL_TAG_RED_RIGHT extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(33, -62, Math.toRadians(270));
        robot.driveSubsystem.setPoseEstimate(startPose);

        robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
        robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
        sleep(1500);
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

        TrajectorySequence toParkingOne = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntake.end())
                .forward(2)
                .lineToLinearHeading(new Pose2d(10, -20, Math.toRadians(90)))
                .build();

        TrajectorySequence toParkingTwo = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntake.end())

                .lineToLinearHeading(new Pose2d(37, -20, Math.toRadians(90)))
                .build();

        TrajectorySequence toParkingThree = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntake.end())
                .forward(2)
                .lineToLinearHeading(new Pose2d(60, -20, Math.toRadians(90)))
                .build();







        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            robot.reset();

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


//        if (tagOfInterest == null) {
//            //default path
//        } else if (tagOfInterest.id == 16) {
//            CommandScheduler.getInstance().reset();
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//                            new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload),
//                            new AutoCycleCommandV6(robot, toPoleAfterConeIntake, toConeStackAfterPoleDeposit, AutoConstants.SLIDE_HEIGHTS[1]),
//                            new WaitCommand(150),
//                            new AutoCycleCommandV6(robot, toPoleAfterConeIntake, toConeStackAfterPoleDeposit, AutoConstants.SLIDE_HEIGHTS[2]),
//                            new WaitCommand(150),
//                            new AutoCycleCommandV6(robot,  toPoleAfterConeIntake, toParkingOne, 0)
//                    )
//
//            );
//        } else if (tagOfInterest.id == 14) {
//            CommandScheduler.getInstance().reset();
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//                            new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload),
//                            new AutoCycleCommandV6(robot, toPoleAfterConeIntake, toConeStackAfterPoleDeposit, AutoConstants.SLIDE_HEIGHTS[1]),
//                            new WaitCommand(150),
//                            new AutoCycleCommandV6(robot, toPoleAfterConeIntake, toConeStackAfterPoleDeposit, AutoConstants.SLIDE_HEIGHTS[2]),
//                            new WaitCommand(150),
//                            new AutoCycleCommandV6(robot,  toPoleAfterConeIntake, toParkingThree, 0)
//                    )
//
//            );
//        } else if (tagOfInterest.id == 19) {
//            CommandScheduler.getInstance().reset();
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//                            new AutoPreloadCommand(robot, toPolePreload, toConeStackAfterPreload),
//                            new AutoCycleCommandV6(robot, toPoleAfterConeIntake, toConeStackAfterPoleDeposit, AutoConstants.SLIDE_HEIGHTS[1]),
//                            new WaitCommand(150),
//                            new AutoCycleCommandV6(robot, toPoleAfterConeIntake, toConeStackAfterPoleDeposit, AutoConstants.SLIDE_HEIGHTS[2]),
//                            new WaitCommand(150),
//                            new AutoCycleCommandV6(robot,  toPoleAfterConeIntake, toParkingTwo, 0)
//                    )
//
//            );
//        }


        robot.reset();

        while (opModeIsActive()) {
            robot.read();

            robot.intake.loop();
            robot.lift.loop();

            robot.write();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();





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