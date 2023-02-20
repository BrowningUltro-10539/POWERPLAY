//package org.firstinspires.ftc.teamcode.commands.Auto;
//
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//public class AutoCycleV3Command extends SequentialCommandGroup {
//    public AutoCycleV3Command(Robot robot, double armPos){
//        super(
//                new ParallelCommandGroup(
//                        new LiftPositionCommand(robot.lift, 1200, 10).alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.TRANSITION))),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.STRAIGHT))
//                ),
//                new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.OUTTAKE)),
//                new WaitCommand(350),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.INTAKE)),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> robot.intake.setArm(armPos)),
//                                new WaitCommand(200),
//                                new InstantCommand(()-> robot.intake.update(IntakeSubsystem.ClawState.CLOSED))
//                        ),
//                        new LiftPositionCommand(robot.lift, 0, 10)
//                ),
//                new WaitCommand(175),
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                        new WaitCommand(175),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.ALIGNED))
//                ),
//                new WaitCommand(375),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(375),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.STRAIGHT)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION))
//                ));
//    }
//}
