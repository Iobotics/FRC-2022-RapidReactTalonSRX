// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public static final class RobotMap{

        //drivetrain devices(motors)
        public static final int kLeftMaster = 0;
        public static final int kRightMaster = 2;
        public static final int kLeftSlave = 1  ;
        public static final int kRightSlave = 3;

        //intake devices(motors)
        public static final int kSpinner = 7;

        //shooter devices(motors)
        public static final int kshootLeft = 8;
        public static final int kshootRight = 9;
    
        }
        
        /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    static final Gains kGains = new Gains(0.000000001, 0.001, 0.0001, 0.2, 0, 1.0);
        //intake devices(motors)
    
    public static final class OIConstants{
        public static final int kJoystick1 = 0;
        public static final int kJoystick2 = 1;
        }

        public static final class DrivetrainConstants {
            public static final double kGearRatio = 1;
            public static final double kWheelDiameter = 6.00;        //in inches
    
            //PID Values
            public static final float kP = 0.05f;
            public static final float kI = 0f;
            public static final float kD = 0f;
            
            //Feed Forward
            public static final float kF = 0.0457f;
        }
    }
