package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting;
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
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleV3Command;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Intake.AutoStackCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Outtake.AutoDepositCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//@TeleOp
//public class CycleAutoCommandTesting extends CommandOpMode {
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
//        if(gamepad1.dpad_left){
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.LEFT_POLE)));
//        }
//
//        if(gamepad1.dpad_right){
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.RIGHT_POLE)));
//        }
//
//        if(gamepad1.dpad_down){
//            schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT)));
//        }
//
//        if(gamepad1.x){
//            schedule(new AutoCycleV3Command(robot, robot.intake.fiveStack[0]));
//        }
//
//
//
//        if(gamepad1.a){
//            schedule(new ParallelCommandGroup(
//                    new AutoStackCommand(robot, robot.intake.fiveStack[0]),
//                    new AutoDepositCommand(robot)
//            ));
//        }
//
//
////            schedule(new SequentialCommandGroup(
////                        new  AutoCycleCommand(robot, robot.intake.fiveStack[0]),
////                        new WaitCommand(1000),
////                        new AutoCycleCommand(robot, robot.intake.fiveStack[1]),
////                        new WaitCommand(1000),
////                        new AutoCycleCommand(robot, robot.intake.fiveStack[2]),
////                        new WaitCommand(1000),
////                        new AutoCycleCommand(robot, robot.intake.fiveStack[3]),
////                        new WaitCommand(1000),
////                        new AutoCycleCommand(robot, robot.intake.fiveStack[4]),
////                        new WaitCommand(1000),
////                        new AutoDepositCommand(robot)
////            ));
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//                            new AutoCycleV2Command(robot, robot.intake.fiveStack[0]),
//                            new WaitCommand(100),
//                            new AutoCycleV2Command(robot, robot.intake.fiveStack[1]),
//                            new WaitCommand(100),
//                            new AutoCycleV2Command(robot, robot.intake.fiveStack[2]),
//                            new WaitCommand(100),
//                            new AutoCycleV2Command(robot, robot.intake.fiveStack[3]),
//                            new WaitCommand(100),
//                            new AutoCycleV2Command(robot, robot.intake.fiveStack[4]),
//                            new WaitCommand(100),
//                            new LiftPositionCommand(robot.lift, 1200, 10).alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.TRANSITION))),
//                            new WaitCommand(300),
//                            new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.OUTTAKE)),
//                            new WaitCommand(400),
//                            new LiftPositionCommand(robot.lift, 0, 10) .alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.INTAKE))),
//                                    new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT))
//                    )
//            );
//
//
//
//
//        if(gamepad1.left_bumper){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)));
//        }
//
//        if(gamepad1.right_bumper){
//            schedule(new InstantCommand(() -> robot.intake.setArm(0.8)));
//        }
//
////        if(gamepad1.x){
////            schedule(new SequentialCommandGroup(
////                    new AutoStackCommand(robot, robot.intake.fiveStack[0]),
////                    new WaitCommand(500),
////                    new AutoStackCommand(robot, robot.intake.fiveStack[1]),
////                    new WaitCommand(500),
////                    new AutoStackCommand(robot, robot.intake.fiveStack[2]),
////                    new WaitCommand(500),
////                    new AutoStackCommand(robot, robot.intake.fiveStack[3]),
////                    new WaitCommand(500),
////                    new AutoStackCommand(robot, robot.intake.fiveStack[4])
////            ));
////        }
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
//        telemetry.addData("Lift Turret Power", robot.lift.turretPower);
//        loopTime = loop;
//        telemetry.update();
//
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();
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
