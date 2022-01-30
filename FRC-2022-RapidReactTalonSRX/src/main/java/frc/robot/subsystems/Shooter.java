// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Shooter extends SubsystemBase{

    private CANSparkMax shootLeft;
    private CANSparkMax shootRight;

    public Shooter(){
        
        shootLeft = new CANSparkMax(RobotMap.kshootLeft, MotorType.kBrushless);
        shootRight = new CANSparkMax(RobotMap.kshootRight, MotorType.kBrushless);

        shootRight.follow(shootLeft);

    }

    public void setPercent(double percent){

     shootLeft.set(percent);
    }

    public void stop(){
        shootLeft.set(0);
    }
    
}