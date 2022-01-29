// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase{

    private WPI_TalonSRX leftMaster;
    private WPI_TalonSRX rightMaster;
    private WPI_TalonSRX leftSlave;
    private WPI_TalonSRX rightSlave;

    public DifferentialDrive drive;

    public Drivetrain(){
        leftMaster = new WPI_TalonSRX(RobotMap.kLeftMaster);
        rightMaster = new WPI_TalonSRX(RobotMap.kRightMaster);
        leftSlave = new WPI_TalonSRX(RobotMap.kLeftSlave);
        rightSlave = new WPI_TalonSRX(RobotMap.kRightSlave);
        drive = new DifferentialDrive(leftMaster, rightMaster);
        
        leftMaster.setInverted(false);
        leftSlave.setInverted(false);
        rightMaster.setInverted(false);
        rightSlave.setInverted(false);
        
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);


    }
}
