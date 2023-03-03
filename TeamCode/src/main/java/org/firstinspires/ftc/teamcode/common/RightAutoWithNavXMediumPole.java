package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV6;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoCycleCommandV7Medium;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoPreloadCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.Cycle.AutoPreloadCommandV2Medium;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.rr.util.AutoConstants;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "RIGHT_OFFICIAL_AUTO_MEDIUM", group = "COMPETITION")
@Disabled
public class RightAutoWithNavXMediumPole extends LinearOpMode {
    private Robot robot;
    private OpenCvCamera camera;
    private SleeveDetection pipeline = new SleeveDetection();
    private TrajectorySequence parkingSeq;

    private double loopTime;

    private Pose2d startPose = new Pose2d(36, -62, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, true);


        robot.driveSubsystem.setPoseEstimate(startPose);

        robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        robot.intake.update(IntakeSubsystem.ArmState.AUTO_INIT);
        robot.intake.update(IntakeSubsystem.RotateState.TRANSFER);
        sleep(1500);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);


//        TrajectorySequence toPolePreload = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
//                .lineTo(new Vector2d(36.2, -4.5))
//                .turn(Math.toRadians(60))
//                .back(1.25)
//                .build();
//
//        Trajectory toConeStackAfterPreload = robot.driveSubsystem.trajectoryBuilder(toPolePreload.end())
//                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
//                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X, AutoConstants.CONE_STACK_LINE_Y))
//                .build();
//
//        TrajectorySequence toPoleAfterConeIntakeCycle1 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPreload.end())
//                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
//                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X, AutoConstants.POLE_SPLINE_Y), Math.toRadians(AutoConstants.POLE_HEADING))
//                .build();
//
//        TrajectorySequence toConeStackAfterPoleDeposit = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle1.end())
//                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
//                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X, AutoConstants.CONE_STACK_LINE_Y))
//                .build();
//
//        TrajectorySequence toPoleAfterConeIntake = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPoleDeposit.end())
//                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
//                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X, AutoConstants.POLE_SPLINE_Y), Math.toRadians(AutoConstants.POLE_HEADING))
//                .build();
//
//        TrajectorySequence toConeStackAfterPoleDepositCycle2 = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle1.end())
//                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
//                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X + 1, AutoConstants.CONE_STACK_LINE_Y))
//                .build();
//
//        TrajectorySequence toPoleAfterConeIntakeCycle2 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPoleDepositCycle2.end())
//                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
//                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X + 1.5, AutoConstants.POLE_SPLINE_Y + 1.5), Math.toRadians(AutoConstants.POLE_HEADING - 1))
//                .build();
//
//        TrajectorySequence toConeStackAfterPoleDepositCycle3 = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle2.end())
//                .splineTo(new Vector2d(AutoConstants.CONE_STACK_SPLINE_X, AutoConstants.CONE_STACK_SPLINE_Y), AutoConstants.CONE_STACK_HEADING)
//                .lineTo(new Vector2d(AutoConstants.CONE_STACK_LINE_X + 3, AutoConstants.CONE_STACK_LINE_Y))
//                .build();
//
//        TrajectorySequence toPoleAfterConeIntakeCycle3 = robot.driveSubsystem.trajectorySequenceBuilder(toConeStackAfterPoleDepositCycle3.end())
//                .lineTo(new Vector2d(AutoConstants.POLE_LINE_X, AutoConstants.POLE_LINE_Y))
//                .splineTo(new Vector2d(AutoConstants.POLE_SPLINE_X + 4, AutoConstants.POLE_SPLINE_Y + 3.5), Math.toRadians(AutoConstants.POLE_HEADING - 1))
//                .build();
//
//        TrajectorySequence toParkingSpotOne = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
//                .forward(5)
//                .lineToLinearHeading(new Pose2d(20, -12, Math.toRadians(270)))
//                .build();
//
//        TrajectorySequence toParkingSpotTwo = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
//                .forward(3)
//                .lineToLinearHeading(new Pose2d(46, -12, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence toParkingSpotThree = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle3.end())
//                .forward(2)
//                .lineToLinearHeading(new Pose2d(65, -10, Math.toRadians(270)))
//
//                .build();



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
                .splineTo(new Vector2d(34, -17.5), Math.toRadians(207))
                .build();

        TrajectorySequence toConeStackAfterConeDepositCycle2 = robot.driveSubsystem.trajectorySequenceBuilder(toPoleAfterConeIntakeCycle1.end())
                .splineTo(new Vector2d(48, -11), Math.toRadians(0))
                .lineTo(new Vector2d(64.75 + 0.2, -11 + 2.25))
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





        while(!isStarted()){
            robot.read();
            robot.write();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new AutoPreloadCommandV2Medium(robot, toPolePreloadMedium, toConeStack),
                        new WaitCommand(10),
                        new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle1, toConeStackAfterConeDepositCycle2, AutoConstants.SLIDE_HEIGHTS[1]),
                                new WaitCommand(10),
                                new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle2, toConeStackAfterConeDepositCycle3, AutoConstants.SLIDE_HEIGHTS[2]),
                                new WaitCommand(10),
                                new AutoCycleCommandV7Medium(robot, toPoleAfterConeIntakeCycle3, toConeStackAfterConeDepositCycle3, AutoConstants.SLIDE_HEIGHTS[3])

                        )
                );


        waitForStart();

        robot.reset();

        while(opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.intake.loop();
            robot.lift.loop();

            robot.write();

            telemetry.addData("Current Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();

//            PhotonCore.CONTROL_HUB.clearBulkCache();

        }






    }
}
