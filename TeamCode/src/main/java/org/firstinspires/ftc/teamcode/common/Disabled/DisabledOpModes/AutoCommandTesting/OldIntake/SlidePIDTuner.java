package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting;
//
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleCommand;
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleV2Command;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Intake.AutoStackCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Outtake.AutoDepositCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//@TeleOp
//public class SlidePIDTuner extends CommandOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//
//    @Override
//    public void initialize(){
//        CommandScheduler.getInstance().reset();
//
//        robot = new Robot(hardwareMap, true);
//        robot.reset();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        for(LynxModule module : robot.getControllers()){
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
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
//       if(gamepad1.a){
//           schedule(new LiftPositionCommand(robot.lift, 0, 10));
//       }
//
//       if(gamepad1.b){
//           schedule(new LiftPositionCommand(robot.lift, 500, 10));
//       }
//
//       if(gamepad1.x){
//           schedule(new LiftPositionCommand(robot.lift, 1200, 10));
//       }
//
//
//
//       if(gamepad1.y){
//           schedule(new LiftPositionCommand(robot.lift, 1500, 10));
//       }
//
//
//
//        robot.intake.setLiftTurretState(robot.lift.turretState);
//        robot.intake.setLiftTurretCurrentAngle(robot.lift.getTurretAngle());
//
//        robot.intake.loop();
//        robot.lift.loop();
//
//
//
//        CommandScheduler.getInstance().run();
//
//        robot.write();
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("Lift Pos", robot.lift.getLiftPos());
//        telemetry.addData("Lift Target Pos", robot.lift.liftTargetPosition);
//        telemetry.addData("Lift Turret Power", robot.lift.liftPower);
//
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
//
