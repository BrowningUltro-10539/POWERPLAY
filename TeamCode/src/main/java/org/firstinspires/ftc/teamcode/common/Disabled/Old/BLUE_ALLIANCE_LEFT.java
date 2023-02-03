//package org.firstinspires.ftc.teamcode.common.Disabled;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
//
//@Autonomous
//@Disabled
//public class BLUE_ALLIANCE_LEFT extends LinearOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        CommandScheduler.getInstance().reset();
//        Robot robot = new Robot(hardwareMap, true);
//
//        //
//
//        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(270));
//        robot.driveSubsystem.setPoseEstimate(startPose);
//
//        Trajectory traj = robot.driveSubsystem.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(35, 23))
//                .splineTo(new Vector2d(47, 12), Math.toRadians(0))
//                .build();
//
//        while(!isStarted()) {
//            robot.read();
//
//            for(LynxModule module : robot.getControllers()){
//                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            }
//
//
//
//            robot.intake.loop();
//            robot.lift.loop();
//
//            robot.write();
//        }
//
//
//        waitForStart();
//
//        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                new TrajectoryFollowerCommand(robot.driveSubsystem, traj)
//        ));
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