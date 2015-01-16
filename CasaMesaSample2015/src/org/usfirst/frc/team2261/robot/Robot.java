/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2261.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends SampleRobot {
    
    // Create a new RobotDrive, composed of the following Jaguar motor controllers
	Talon frontRightController = new Talon(0);
	Talon rearRightController = new Talon(1);
	Talon frontLeftController = new Talon(2);
	Talon rearLeftController = new Talon(3);
    RobotDrive robotDrive = new RobotDrive(frontLeftController, rearLeftController,
                                            frontRightController, rearRightController);
    
    // New variable here
	Talon talonMC9 = new Talon(9);
    
    // Create new Joystick on ports 1 and 2
    Joystick driverJoystick = new Joystick(0);
    Joystick accessoryJoystick = new Joystick(1);

    boolean robotInitted = false;
    boolean useForklift = false;
    
    protected void robotInit() {
        // (Re-)Enable motor safety with .25s expiration
        // "Motor safety is a mechanism built into the RobotDrive object that
        // will turn off the motors if the program doesn't continuously update
        // the motor speed." (default "expiration" is 100ms)
        robotDrive.setExpiration(0.025);  
        robotDrive.setSafetyEnabled(true);
        
        // Set Talon motor
        talonMC9.setExpiration(0.025);
        talonMC9.setSafetyEnabled(true);
        
        // Reverse left side drive
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        
        // Ensure all motors are stopped (I don't believe we should have to do this)
        robotDrive.drive(0, 0);

        robotInitted = true;
    }

    public void autonomous() {
        // Autonomous here        
    }
    
    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        robotDrive.setExpiration(0.025);  
      
        if(!robotInitted) {
          System.out.println("Robot not initted? Initting...");
          robotInit();
          robotInitted = true;            
        }
        
        // While still under operator control and enabled, "mecanum drive" robot,
        // updating motors every 1/100th second
        double updateDelay = 0.0075; // How often to refresh
        while(isOperatorControl() && isEnabled()) {
          
            // Drive the Robot
            // "This is designed to be directly driven by joystick axes. "
//            robotDrive.mecanumDrive_Cartesian(scaleJoystickInput(driverJoystick.getX()),
//                                                scaleJoystickInput(driverJoystick.getY()), 
//                                                scaleJoystickInput(driverJoystick.getZ()), 0);
            
        	talonMC9.set(scaleJoystickInput(driverJoystick.getY()));
            
            // Output debug information
        	SmartDashboard.putNumber("driverJoystick.getY()", driverJoystick.getY());
//            putSmartDashboardValues();
            
            Timer.delay(updateDelay); // Wait the specified second before updating again
        }
        
        // Stop the robot
        robotDrive.drive(0, 0);
    }

	private void putSmartDashboardValues() {
		SmartDashboard.putNumber("driverJoystick.getX()", driverJoystick.getX());
		SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getX())",
		                            scaleJoystickInput(driverJoystick.getX()));
		SmartDashboard.putNumber("driverJoystick.getY()",driverJoystick.getY());
		SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getY())",
		                            scaleJoystickInput(driverJoystick.getY()));
		SmartDashboard.putNumber("driverJoystick.getZ()",driverJoystick.getZ());
		SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getZ())",
		                            scaleJoystickInput(driverJoystick.getZ()));
		SmartDashboard.putNumber("driverJoystick.getThrottle()", driverJoystick.getThrottle());
		SmartDashboard.putNumber("accessoryJoystick.getX())",accessoryJoystick.getX());
		SmartDashboard.putNumber("accessoryJoystick.getY()",accessoryJoystick.getY());
		SmartDashboard.putNumber("scaleJoystickInput(accessoryJoystick.getY())",
		                            scaleJoystickInput(accessoryJoystick.getX()));
		SmartDashboard.putNumber("accessoryJoystick.getZ()",accessoryJoystick.getZ());
		SmartDashboard.putNumber("accessoryJoystick.getThrottle()", accessoryJoystick.getThrottle());
		SmartDashboard.putNumber("accessoryJoystick.getTwist()",accessoryJoystick.getTwist());
		SmartDashboard.putBoolean("accessoryJoystick.getRawButton(2)",accessoryJoystick.getRawButton(2));
		SmartDashboard.putBoolean("accessoryJoystick.getRawButton(3)",accessoryJoystick.getRawButton(3));
	}

    protected void disabled() {
        robotDrive.drive(0, 0);
        robotInitted = false;
    }
    
    /**
     * This method leverages a heuristic to "efficiently" add a dead zone
     *  and scale the input to a quadratic curve
     * @param input value to be scaled, assumed to be [-1,1]
     * @return scaled input
     */
    public double scaleJoystickInput(double input) {
        // Scales input from [-1,1] to [-0.03,0.97]
        double adjustedInput = input * input - 0.03;
        if(adjustedInput < 0) { // Interpret < 0 as deadzone
            adjustedInput = 0;
        } else if(adjustedInput > 0.9) { // Interpret anything near max as max
            adjustedInput = 1;
        }
        
        if(input < 0) { // Adjust sign to match original input
            adjustedInput = -adjustedInput;
        }
        
        return adjustedInput;
    }
}
