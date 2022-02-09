// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase{

    private WPI_TalonFX leftMaster;
    private WPI_TalonFX rightMaster;
    private WPI_TalonFX leftSlave;
    private WPI_TalonFX rightSlave;

    public DifferentialDrive drive;

    public Drivetrain(){
        leftMaster = new WPI_TalonFX(RobotMap.kLeftMaster);
        rightMaster = new WPI_TalonFX(RobotMap.kRightMaster);
        leftSlave = new WPI_TalonFX(RobotMap.kLeftSlave);
        rightSlave = new WPI_TalonFX(RobotMap.kRightSlave);
        drive = new DifferentialDrive(leftMaster, rightMaster);
        
        leftMaster.setInverted(false);
        leftSlave.setInverted(false);
        rightMaster.setInverted(false);
        rightSlave.setInverted(false);
        
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
    }
    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.set(ControlMode.PercentOutput, 0);
      }

      public void motionMagic (double distance, double speed) {
        double rotations = (distance * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
        double targetPos = rotations*2048;
        //Convert target speed from inches / second to encoder units / 100 ms
        double targetSpeed = (speed *DrivetrainConstants.kGearRatio * 2048 * 10) / (DrivetrainConstants.kWheelDiameter * Math.PI);
    
        rightSlave.follow(leftMaster);
        rightMaster.follow(leftMaster);
        leftMaster.configMotionCruiseVelocity((int)targetSpeed);
        leftMaster.configMotionAcceleration((int)targetSpeed);
        leftMaster.setSelectedSensorPosition(0);
        leftMaster.set(ControlMode.MotionMagic, targetPos);
      }
    
      //Are we there yet
      public boolean isTargetAchieved (double distance, double error) {
        double rotations = (distance * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
        double targetPos = rotations*2048;
        //converting allowed error from inches to encoder units
        double allowedError = ((error * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter * Math.PI) * 2048);
        if(Math.abs(leftMaster.getSelectedSensorPosition() - targetPos) <= allowedError && leftMaster.getSelectedSensorVelocity() == 0.0 && leftMaster.getActiveTrajectoryVelocity() < 3) {
          return true;
        } else{
          return false;
        }
    }
        public void config () {
            
          }     
}
