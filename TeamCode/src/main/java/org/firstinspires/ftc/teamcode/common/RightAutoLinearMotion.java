package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV6;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "RIGHT_OFFICIAL_AUTO_WITH_TURN", group = "COMPETITION")
@Disabled
public class RightAutoLinearMotion extends LinearOpMode {
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

//        Trajectory toPolePreload = robot.driveSubsystem.trajectoryBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(36, -16, Math.toRadians(270)))
//                .splineTo(new Vector2d(35.9, -5), Math.toRadians(150))
//                .build();
//
//        Trajectory toConeStackAfterPreload = robot.driveSubsystem.trajectoryBuilder(toPolePreload.end())
//                .splineTo(new Vector2d(45, -13), Math.toRadians(0))
//                .lineTo(new Vector2d(63, -13))
//                .build();
//
//        Trajectory toPoleAfterConeStack = robot.driveSubsystem.trajectoryBuilder(toConeStackAfterPreload.end())
//                .lineToSplineHeading(new Pose2d(45, -12, Math.toRadians(0)))
//                .splineTo(new Vector2d(35.9, -5), Math.toRadians(150))
//                .build();
//
//        Trajectory toConeStackAfterPole = robot.driveSubsystem.trajectoryBuilder(toPoleAfterConeStack.end())
//                .splineTo(new Vector2d(45, -13), Math.toRadians(0))
//                .lineTo(new Vector2d(63, -13))
//                .build();

        TrajectorySequence toPoleForPreload = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36.2, -5))
                .turn(Math.toRadians(65))
                .back(0.75)
                .build();

        TurnCommand turnToPole = new TurnCommand(robot.driveSubsystem, 65);

        Trajectory backUpToPole = robot.driveSubsystem.trajectoryBuilder(toPoleForPreload.end())
                .back(1)
                .build();

        TrajectorySequence toConeStack = robot.driveSubsystem.trajectorySequenceBuilder(toPoleForPreload.end())
                .lineTo(new Vector2d(48, -12.5))
                .turn(Math.toRadians(25))
                .build();

        TurnCommand turnToConeStack = new TurnCommand(robot.driveSubsystem, -65);

        Trajectory toFirstCone = robot.driveSubsystem.trajectoryBuilder(toConeStack.end())
                .lineTo(new Vector2d(62.5, -12.5))
                .build();

        TrajectorySequence toPoleToDepositCone = robot.driveSubsystem.trajectorySequenceBuilder(toFirstCone.end())
                .lineTo(new Vector2d(36.2, -5))
                .turn(Math.toRadians(-25))
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


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPoleForPreload),
                                new SequentialCommandGroup(
                                        new WaitCommand(1750),
                                        new ParallelCommandGroup(
                                                new NewLiftPositionCommand(robot.lift, 20.5, 200, 200, 2),
                                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT))
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new NewLiftPositionCommand(robot.lift, 18.5, 200, 200, 2),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.POLE_DUNK))
                        ),
                        new WaitCommand(150),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                            new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toConeStack),
                            new SequentialCommandGroup(
                                    new WaitCommand(200),
                                    new ParallelCommandGroup(
                                            new NewLiftPositionCommand(robot.lift, 3.5, 200, 200, 2),
                                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                                    )
                            )
                        ),
                        new WaitCommand(150),
                        new TrajectoryFollowerCommand(robot.driveSubsystem, toFirstCone),
                        //Start to get first cone
                        new WaitCommand(150),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(500),
                        new NewLiftPositionCommand(robot.lift, 9, 200, 200, 2),
                        new ParallelCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPoleToDepositCone),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER)),
                                        new WaitCommand(1500),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                                        new NewLiftPositionCommand(robot.lift, 21, 200, 200, 2)
                                )
                        ),
                        new ParallelCommandGroup(
                                new NewLiftPositionCommand(robot.lift, 17, 200, 200, 2),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.POLE_DUNK)),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)
                                )

                        )),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toConeStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ParallelCommandGroup(
                                                new NewLiftPositionCommand(robot.lift, 2.75, 200, 200, 2),
                                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                                        )
                                )
                        ),
                        new WaitCommand(150),
                        new TrajectoryFollowerCommand(robot.driveSubsystem, toFirstCone),
                        //Second Cone
                        new WaitCommand(150),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(500),
                        new NewLiftPositionCommand(robot.lift, 9, 200, 200, 2),
                        new ParallelCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPoleToDepositCone),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER)),
                                        new WaitCommand(1500),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                                        new NewLiftPositionCommand(robot.lift, 21, 200, 200, 2)
                                )
                        ),
                        new ParallelCommandGroup(
                                new NewLiftPositionCommand(robot.lift, 17, 200, 200, 2),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.POLE_DUNK)),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)
                                        )

                                )),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toConeStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ParallelCommandGroup(
                                                new NewLiftPositionCommand(robot.lift, 3.25, 200, 200, 2),
                                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))
                                        )
                                )
                        ),
                        new WaitCommand(150),
                        new TrajectoryFollowerCommand(robot.driveSubsystem, toFirstCone)
        ));

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
