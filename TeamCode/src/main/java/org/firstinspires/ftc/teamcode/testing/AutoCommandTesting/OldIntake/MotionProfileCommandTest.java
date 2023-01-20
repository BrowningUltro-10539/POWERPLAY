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
//import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakePositionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//@TeleOp
//public class MotionProfileCommandTest extends CommandOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//
//
//
//    @Override
//    public void initialize(){
//        CommandScheduler.getInstance().reset();
//
//        robot = new Robot(hardwareMap, false);
//        robot.reset();
//
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
//            schedule(new IntakePositionCommand(robot.intake, 0, 2000, 3000, 2, 2000));
//        }
//
//        if(gamepad1.b){
//            schedule(new IntakePositionCommand(robot.intake, 50, 2000, 3000, 2, 2000));
//        }
//
//        if(gamepad1.x){
//            schedule(new IntakePositionCommand(robot.intake, -50, 2000, 3000, 2, 2000));
//        }
//
//        if(gamepad1.dpad_down){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)));
//        }
//
//
//        robot.intake.loop();
//        robot.lift.loop();
//
//        CommandScheduler.getInstance().run();
//
//
//        robot.write();
//
//
//        double loop = System.nanoTime();
//        telemetry.addData("Loop Time: ", 1000000000 / (loop - loopTime));
//        telemetry.addData("Turret Position: ", robot.intake.getPos());
//        telemetry.addData("Power: ", robot.intake.power);
//        telemetry.addData("Target Pos:  ", robot.intake.targetPosition);
//        telemetry.addData("Color: ", robot.intake.sensorColor);
//        telemetry.addData("Distance: ", robot.intake.sensorDistance);
//        telemetry.update();
//
//
//
//        loopTime = loop;
//
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();
//    }
//
//
//}
