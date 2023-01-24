//package org.firstinspires.ftc.teamcode.commands.Auto.Intake;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//public class IntakeTransferCommand extends SequentialCommandGroup {
//    public IntakeTransferCommand(Robot robot){
//        super(
//             new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)).alongWith(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.TRANSFER))).alongWith(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.ALIGNED))),
//                new WaitCommand(2000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(2000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)).alongWith(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.STRAIGHT))).alongWith(new InstantCommand(()-> robot.intake.update(IntakeSubsystem.IntakeState.INTAKE)))
//        );
//    }
//}
