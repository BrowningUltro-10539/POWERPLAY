package org.firstinspires.ftc.teamcode.commands.IntakeCommands;//package org.firstinspires.ftc.teamcode.commands.IntakeCommands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//public class IntakeFiveStackIntakeAndDepositCommand extends SequentialCommandGroup {
//    public IntakeFiveStackIntakeAndDepositCommand(Robot robot){
//        super(
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(250),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)),
//                new WaitCommand(2000),
//                new InstantCommand(() -> robot.intake.setArm(robot.intake.fiveStack[1])),
//                new WaitCommand(1000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(250),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)),
//                new WaitCommand(2000),
//                new InstantCommand(() -> robot.intake.setArm(robot.intake.fiveStack[2])),
//                new WaitCommand(1000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(250),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)),
//                new WaitCommand(2000),
//                new InstantCommand(() -> robot.intake.setArm(robot.intake.fiveStack[3])),
//                new WaitCommand(1000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(250),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)),
//                new WaitCommand(2000),
//                new InstantCommand(() -> robot.intake.setArm(robot.intake.fiveStack[4])),
//                new WaitCommand(1000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(250),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(500),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)),
//                new WaitCommand(2000)
//                );
//    }
//}
