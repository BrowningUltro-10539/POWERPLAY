package org.firstinspires.ftc.teamcode.testing.DisabledOpModes.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.testing.DisabledOpModes.AutoCommandTesting;
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
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleV2Command;
//import org.firstinspires.ftc.teamcode.commands.Auto.AutoCycleV3Command;
//import org.firstinspires.ftc.teamcode.commands.OldCommandStuff.Intake.AutoStackCommand;
//import org.firstinspires.ftc.teamcode.commands.OldCommandStuff.Outtake.AutoDepositCommand;
//import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//@Autonomous
//public class CycleTester extends CommandOpMode {
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
//            module.clearBulkCache();
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
//        if(opModeIsActive()){
//            schedule(new AutoCycleV2Command(robot, robot.intake.fiveStack[0]));
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
