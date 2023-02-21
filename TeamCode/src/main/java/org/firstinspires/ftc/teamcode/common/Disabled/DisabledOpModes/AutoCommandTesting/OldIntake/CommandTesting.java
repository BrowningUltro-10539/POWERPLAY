package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting;
//
//import com.acmerobotics.dashboard.FtcDashboard;
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
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Intake.AutoStackCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.commands.subsystem.LiftSubsystem;
//
//@TeleOp
//public class CommandTesting extends CommandOpMode {
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
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)));
//        }
//
//
//
//        if(gamepad1.x){
//            schedule(new SequentialCommandGroup(
//                    new AutoStackCommand(robot, robot.intake.fiveStack[0]),
//                    new WaitCommand(500),
//                    new AutoStackCommand(robot, robot.intake.fiveStack[1]),
//                    new WaitCommand(500),
//                    new AutoStackCommand(robot, robot.intake.fiveStack[2]),
//                    new WaitCommand(500),
//                    new AutoStackCommand(robot, robot.intake.fiveStack[3]),
//                    new WaitCommand(500),
//                    new AutoStackCommand(robot, robot.intake.fiveStack[4])
//                    ));
//        }
//
//        if(gamepad1.dpad_up){
//            schedule(new LiftPositionCommand(robot.lift, 800, 10));
//        }
//
//        if(gamepad1.dpad_down){
//            schedule(new LiftPositionCommand(robot.lift, 0, 10));
//        }
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
//        loopTime = loop;
//        telemetry.update();
//
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();
//    }
//
//
//}
