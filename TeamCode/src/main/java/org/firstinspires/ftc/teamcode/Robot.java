package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDriveWithNavX;
import org.firstinspires.ftc.teamcode.rr.drive.localizers.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystem.MecanumDriveSubsystem;

import java.util.List;

public class Robot {


    public StandardTrackingWheelLocalizer localizer;
    public MecanumDriveSubsystem driveSubsystem;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;

    private boolean isAuto = false;

    public Pose2d robotPose;

    private List<LynxModule> controllers;

    public Robot(HardwareMap hardwareMap, boolean isAuto){
        this.isAuto = isAuto;

        localizer = new StandardTrackingWheelLocalizer(hardwareMap);

        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDriveWithNavX(hardwareMap), false);
        intake = new IntakeSubsystem(hardwareMap, isAuto);
        lift = new LiftSubsystem(hardwareMap, isAuto);

        controllers = hardwareMap.getAll(LynxModule.class);

    }

    public Robot(HardwareMap hardwareMap){
        this(hardwareMap, false);
    }

    public void read(){
        intake.read();
        lift.read();
        robotPose = localizer.getPose();
    }

    public void write(){
        intake.write();
        lift.write();
        driveSubsystem.update();
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
