package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystem.MecanumDriveSubsystem;

import java.util.List;

public class Robot {
    
    public MecanumDriveSubsystem driveSubsystem;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;

    private boolean isAuto = false;

    public Pose2d robotPose;

    private List<LynxModule> controllers;

    public Robot(HardwareMap hardwareMap, boolean isAuto){
        this.isAuto = isAuto;



        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        intake = new IntakeSubsystem(hardwareMap, isAuto);
        lift = new LiftSubsystem(hardwareMap, isAuto);

        if(isAuto){
            lift.lift2.encoder.reset();
        }

        controllers = hardwareMap.getAll(LynxModule.class);

    }

    public Robot(HardwareMap hardwareMap){
        this(hardwareMap, false);
    }

    public void read(){
        intake.read();
        lift.read();

        if(isAuto){
            robotPose = driveSubsystem.getPoseEstimate();
        }

    }

    public void write(){
        intake.write();
        lift.write();

        if(isAuto){
            driveSubsystem.update();
        }

    }

    public void reset(){
        lift.lift1.resetEncoder();
    }

    public List<LynxModule> getControllers(){
        return controllers;
    }

    public Pose2d getPose(){
        return robotPose;
    }





}
