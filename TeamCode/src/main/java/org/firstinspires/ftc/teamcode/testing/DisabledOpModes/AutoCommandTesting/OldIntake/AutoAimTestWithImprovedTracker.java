package org.firstinspires.ftc.teamcode.testing.DisabledOpModes.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.testing.DisabledOpModes.AutoCommandTesting;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.OldCommandStuff.IntakeCommands.ColorSensorTestCommand;
//import org.firstinspires.ftc.teamcode.commands.OldCommandStuff.IntakeCommands.DepositWColorSensorCommandTest;
//import org.firstinspires.ftc.teamcode.commands.OldCommandStuff.IntakeCommands.IntakeAndDepositCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//
//import org.firstinspires.ftc.teamcode.vision.ConeTracker;
//import org.firstinspires.ftc.teamcode.vision.ImprovedConeTracker;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@TeleOp
//@Config
//public class AutoAimTestWithImprovedTracker extends CommandOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//    OpenCvWebcam webcam;
//    ImprovedConeTracker pipeline;
//
//    double targetAngle = 0;
//    public static double p = -0.01;
//
//    @Override
//    public void initialize(){
//        CommandScheduler.getInstance().reset();
//
//        robot = new Robot(hardwareMap, false);
//        robot.reset();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        pipeline = new ImprovedConeTracker();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(pipeline);
//
//        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened() { webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); }
//
//            @Override
//            public void onError(int errorCode) {}
//        });
//
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(0);
//        PhotonCore.enable();
//
//    }
//
//
//    @Override
//    public void run(){
//        if(timer == null){
//            timer = new ElapsedTime();
//            robot.reset();
//        }
//
//        FtcDashboard.getInstance().startCameraStream(webcam,60);
//
//        robot.read();
//
//
//        targetAngle = -pipeline.getConeCentroid() * p;
//
//        robot.intake.setTurretTargetAngle(targetAngle);
//
//        schedule(new InstantCommand(() -> robot.intake.setArm(0.995)));
//
//        robot.intake.loop();
//        robot.lift.loop();
//
//        CommandScheduler.getInstance().run();
//
//        robot.write();
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("Angle from Pipeline", targetAngle);
//        telemetry.addData("Angle being fed ", targetAngle * p);
//        telemetry.addData("Turret Angle, ", robot.intake.getTurretAngle());
//        telemetry.addData("Turret Target Angle ", robot.intake.getTargetAngle());
//        loopTime = loop;
//        telemetry.update();
//
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();
//    }
//}
//
