// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public static final class RobotMap{

        //Drivetrain devices
        public static final int kLeftMaster = 0;
        public static final int kRightMaster = 2;
        public static final int kLeftSlave = 1  ;
        public static final int kRightSlave = 3;

        //shooter devices
        public static final int kArm = 8;
        public static final int kSpinner = 0;
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
    static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
    
        
    
    public static final class OIConstants{
        public static final int kJoystick1 = 0;
        public static final int kJoystick2 = 1;
        }


}

