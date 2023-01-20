package org.firstinspires.ftc.teamcode.testing.DriveCommandTesting;//package org.firstinspires.ftc.teamcode.testing.DriveCommandTesting;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleV2Command;
//import org.firstinspires.ftc.teamcode.commands.Auto.Intake.AutoStackCommand;
//import org.firstinspires.ftc.teamcode.commands.Auto.Intake.IntakeTransferCommand;
//import org.firstinspires.ftc.teamcode.commands.Auto.Outtake.AutoDepositCommand;
//import org.firstinspires.ftc.teamcode.commands.Auto.Outtake.DepositAndRetractCommand;
//import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//@TeleOp
//@Config
//public class IntakeArmTuner extends LineOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//    public static double target = 0;
//
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
//        for(LynxModule module : robot.getControllers()){
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//
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
//        robot.read();
//
//
//        robot.intake.setArmTargetAngle(target);
//
//
//        robot.write();
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        loopTime = loop;
//        telemetry.update();
//
//        for(LynxModule module : robot.getControllers()){
//            module.clearBulkCache();
//        }
//    }
//
//    @Override
//    public void reset(){
//        schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT)));
//        CommandScheduler.getInstance().reset();
//    }
//
//
//}
