package org.firstinspires.ftc.teamcode.testing.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.testing.AutoCommandTesting;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeAndDepositCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeFiveStackIntakeAndDepositCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//@TeleOp
//public class FiveStackServoPositionTesting extends CommandOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
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
//        robot.read();
//
//        if(gamepad1.a){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)));
//        }
//
//        if(gamepad1.b){
//            schedule(new IntakeFiveStackIntakeAndDepositCommand(robot));
//        }
//
//        if(gamepad1.x){
//            schedule(new InstantCommand(() -> robot.intake.setArm(0.925)));
//        }
//
//        if(gamepad1.y){
//            schedule(new IntakeAndDepositCommand(robot));
//        }
//
//        if(gamepad1.dpad_up){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)));
//        }
//
//        if(gamepad1.dpad_down){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)));
//        }
//
//
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
//        loopTime = loop;
//        telemetry.update();
//
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();
//    }
//
//
//}