package org.firstinspires.ftc.teamcode.common.Disabled;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class RED_ALLIANCE_RIGHT_VERT_TESTING extends LinearOpMode {
    private Robot robot;
    private ElapsedTime timer;
    private double loopTime = 0;

    private OpenCvCamera camera;




    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, true);

        //

        Pose2d startPose = new Pose2d(34, -62, Math.toRadians(270));
        robot.driveSubsystem.setPoseEstimate(startPose);



        TrajectorySequence testing = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(34, -24))
                .splineTo(new Vector2d(32, -5), Math.toRadians(120))
                .waitSeconds(2)
                .splineTo(new Vector2d(62, -11), Math.toRadians(0))
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(32, -5), Math.toRadians(120))
                .setReversed(false)
                .waitSeconds(2)
                .splineTo(new Vector2d(62, -11), Math.toRadians(0))
                .build();


        while(!isStarted()) {
            robot.read();

            for(LynxModule module : robot.getControllers()){
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }







            robot.intake.loop();
            robot.lift.loop();

            robot.write();

            for(LynxModule module : robot.getControllers()){
                module.clearBulkCache();
            }

        }



        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(

                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, testing)

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

