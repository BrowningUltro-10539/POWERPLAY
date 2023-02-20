package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.testing;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
//import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
//
//@Autonomous
//public class RedRightAutoOpMode extends LinearOpMode {
//    public Pose2d startPose = new Pose2d(-36, 50, Math.toRadians(0));
//
//    public TrajectorySequence lineToSplineStarterStack;
//    public SampleMecanumDrive drive;
//
//    @Override
//    public void runOpMode(){
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(startPose);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//
//        lineToSplineStarterStack = drive.trajectorySequenceBuilder(startPose)
//                .lineTo(new Vector2d(10, 50))
//                .splineTo(new Vector2d(10, 40), Math.toRadians(-90))
//                .lineTo(new Vector2d(10, 20))
//                .build();
//
//
//
//            drive.followTrajectorySequence(lineToSplineStarterStack);
//
//    }
//
//
//}
