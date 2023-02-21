package org.firstinspires.ftc.teamcode.common.Disabled;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Old.Testing.AutoDepositAndRetract;
import org.firstinspires.ftc.teamcode.commands.Old.Testing.GrabConeDriveDepositReturnCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
@Disabled
public class RED_ALLIANCE_RIGHT extends LinearOpMode {
    private Robot robot;
    private ElapsedTime timer;
    private double loopTime = 0;

    private OpenCvCamera camera;




    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, true);

        //

        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));
        robot.driveSubsystem.setPoseEstimate(startPose);

        Trajectory traj = robot.driveSubsystem.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(35, -24))
                .build();

        Trajectory toConeStack = robot.driveSubsystem.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(41.5, -11.5), Math.toRadians(360-42))

                .build();

        Trajectory toConeOne = robot.driveSubsystem.trajectoryBuilder(toConeStack.end())
                .lineToLinearHeading(new Pose2d(63, -10, Math.toRadians(360)))
                .build();


        Trajectory toPole = robot.driveSubsystem.trajectoryBuilder(toConeOne.end())
                .lineToLinearHeading(new Pose2d(41.5, -11.5, Math.toRadians(360-32)))
                .build();

        Trajectory toConeTwo = robot.driveSubsystem.trajectoryBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(62.4, -12, Math.toRadians(360)))
                .build();

        Trajectory toConeThree = robot.driveSubsystem.trajectoryBuilder(toPole.end())
                .lineTo(new Vector2d(62, -12))
                .build();

        TrajectorySequence testing = robot.driveSubsystem.trajectorySequenceBuilder(toPole.end())
                .lineTo(new Vector2d(-35, -60))
                .build();


        while(!isStarted()) {
            robot.read();

            for(LynxModule module : robot.getControllers()){
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }


            robot.intake.update(IntakeSubsystem.RotateState.INTAKE);
            robot.intake.update(IntakeSubsystem.ArmState.INTAKE);
            robot.intake.update(IntakeSubsystem.ClawState.OPEN);





            robot.intake.loop();
            robot.lift.loop();

            robot.write();

            for(LynxModule module : robot.getControllers()){
                module.clearBulkCache();
            }

        }



        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(

                //Grab Cone
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))
//                ),
//                new WaitCommand(100),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(100),

                //Drive towards mid point within the cone stack trajectory
                new TrajectoryFollowerCommand(robot.driveSubsystem, traj),



                //Lift Arm
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                new WaitCommand(25),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER)),
                new WaitCommand(50),


                //Drive to coneStack

                new TrajectoryFollowerCommand(robot.driveSubsystem, toConeStack),

//                Drop preload
                new AutoDepositAndRetract(robot, 6.5),

                //Drive to the first cone

                new WaitCommand(500),

                new InstantCommand(() -> robot.driveSubsystem.turn(Math.toRadians(42))),
                new WaitCommand(1500),
                new TrajectoryFollowerCommand(robot.driveSubsystem, toConeOne),
                new WaitCommand(1500),
                new GrabConeDriveDepositReturnCommand(robot, toPole, toConeTwo, 5.75)
//                new WaitCommand(2000),
//                new TrajectoryFollowerCommand(robot.driveSubsystem, toPole),
//                new WaitCommand(2000),
//                new TrajectoryFollowerCommand(robot.driveSubsystem, toConeOne),
//                new WaitCommand(2000),
//                new TrajectoryFollowerCommand(robot.driveSubsystem, toPole),
//                new WaitCommand(2000)


//                new WaitCommand(200),
//                new GrabConeDriveDepositReturnCommand(robot, toPole, toConeTwo, 5.75)
//                new WaitCommand(1000),
//                new GrabConeDriveDepositReturnCommand(robot, toPole, toConeOne, 4.75)


//                new AutoPickUpConeCommand(robot),
//
//                //Drive to drop-off zone
//                new TrajectoryFollowerCommand(robot.driveSubsystem, toPole),
//
//                //Drop off the first cone
//                new AutoDepositAndRetract(robot, 5.75),
//
//                new TrajectoryFollowerCommand(robot.driveSubsystem, toConeTwo),
//                new WaitCommand(500),
//                new AutoPickUpConeCommand(robot),
//
//                //Drive to drop-off zone
//                new TrajectoryFollowerCommand(robot.driveSubsystem, toPole),
//
//                //Drop off the first cone
//                new AutoDepositAndRetract(robot, 5)

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

