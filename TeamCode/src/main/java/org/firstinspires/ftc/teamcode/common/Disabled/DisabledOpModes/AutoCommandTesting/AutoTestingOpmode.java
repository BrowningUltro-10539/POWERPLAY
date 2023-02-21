package org.firstinspires.ftc.teamcode.common.Disabled.DisabledOpModes.AutoCommandTesting;//package org.firstinspires.ftc.teamcode.testing;
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
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleV2Command;
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleV3Command;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Intake.AutoStackCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Outtake.AutoDepositCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.rr.drive.localizers.StandardTrackingWheelLocalizer;
//import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.commands.subsystem.LiftSubsystem;
//
//@Autonomous
//public class AutoTestingOpmode extends LinearOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        CommandScheduler.getInstance().reset();
//        Robot robot = new Robot(hardwareMap, true);
//
//        while(!isStarted()) {
//            robot.read();
//
//            for(LynxModule module : robot.getControllers()){
//                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            }
//
//            robot.write();
//        }
//
//
//        waitForStart();
//
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.RIGHT_POLE)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                        new WaitCommand(1000),
//                        new AutoCycleV2Command(robot, robot.intake.fiveStack[0]),
//                        new WaitCommand(100),
//                        new AutoCycleV2Command(robot, robot.intake.fiveStack[1]),
//                        new WaitCommand(100),
//                        new AutoCycleV2Command(robot, robot.intake.fiveStack[2]),
//                        new WaitCommand(100),
//                        new AutoCycleV2Command(robot, robot.intake.fiveStack[3]),
//                        new WaitCommand(100),
//                        new AutoCycleV2Command(robot, robot.intake.fiveStack[4]),
//                        new WaitCommand(100),
//                        new LiftPositionCommand(robot.lift, 1200, 10).alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.TRANSITION))),
//                        new WaitCommand(300),
//                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.OUTTAKE)),
//                        new WaitCommand(400),
//                        new LiftPositionCommand(robot.lift, 0, 10) .alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.INTAKE))),
//                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT))
//                    )
//                );
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
//
