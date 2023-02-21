//package org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.NewBot;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.commands.subsystem.LiftSubsystem;
//
//public class AutoDepositAndIntakeCommand extends SequentialCommandGroup {
//    public AutoDepositAndIntakeCommand(Robot robot){
//        super(
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT))),
//                new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.RIGHT_POLE)),
//                new LiftPositionCommand(robot.lift, 24, 2),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(200),
//                new SequentialCommandGroup(
//                        new LiftPositionCommand(robot.lift, 0, 2),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
//                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE))
//
//                        )
//
//                )
//        );
//    }
//}
