/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3006.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;


public class Robot extends SampleRobot {
	
	Victor motorleft;
	Victor motorright;
	
	Victor lifter;
	Victor led;
	Victor intake;
	
	Joystick pilotone;
	Joystick pilottwo;
		
	final double deadzone = 0.01;
	
	final int xbuttonone = 1;
	final int abuttonone = 2;
	final int bbuttonone = 3;
	final int ybuttonone = 4;
	final int lbumperone = 5;
	final int rbumperone = 6;
	final int ltriggerone = 7;
	final int rtriggerone = 8;
	
	final int abuttontwo = 1;
	final int bbuttontwo = 2;
	final int xbuttontwo = 3;
	final int ybuttontwo = 4;
	final int lbumpertwo = 5;
	final int rbumpertwo = 6;
	
	final int ryaxisone = 3;
	final int lyaxisone = 1;
	
	final int ryaxistwo = 5;
	final int lyaxistwo = 1;
	final int rxaxistwo = 4;
	
	
	final int rtriggertwo = 3;
	final int ltriggertwo = 2; //THESE ARE TREATED AS AXIS
	
	final double revtofullheight = 42/Math.PI; //42/pi
	final double gearboxration = 0.0048;
	
	Encoder lifterEncoder;

	NetworkTable table;
	NetworkTableEntry vals;
	NetworkTableEntry right;
	NetworkTableEntry left;
	NetworkTableEntry forward;
	
	AnalogInput leftAI;
	AnalogInput rightAI;
	
	double prevSpeedR = 0;
	double prevSpeedL = 0;
	
	public Robot() {
		motorleft = new Victor(0);
		motorright = new Victor(1);
		lifter = new Victor(2);
		intake = new Victor(4);
		led = new Victor(3);
		
		pilotone = new Joystick(0);
		pilottwo = new Joystick(1);
		
		lifterEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		lifterEncoder.setMaxPeriod(revtofullheight);
		
		table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
		right = table.getEntry("right");
		right.setValue(new String("r"));
		left = table.getEntry("left");
		left.setValue(new String("l"));
		forward = table.getEntry("forward");
		forward.setValue(new String("f"));
		vals = table.getEntry("dir");
		
		
		leftAI = new AnalogInput(1);
		rightAI = new AnalogInput(0);
		
	}
	

	
	@Override
	public void autonomous() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(gameData.length() > 0)
		{
			if(gameData.charAt(0) == 'L')
			{
				//Left Auto Code
				if(rightAI.getVoltage() < 4.5)
				{
					Move(-0.25, 0.25);
					Timer.delay(40);
					
					Move(1,1);
					Timer.delay(200);
					
					Move(0.25, -0.25);
					Timer.delay(40);
					
					Move(1,1);
					Timer.delay(200);
					
					Move(0.25, -0.25);
					Timer.delay(40);
				}
				else if(rightAI.getVoltage() >= 4.5 && leftAI.getVoltage() >= 4.5)
				{
					Move(0.25, 0.25);
					Timer.delay(1);
					Move(0.25, -0.25);
					Timer.delay(0.25);
					Move(0.25, 0.25);
					Timer.delay(1);
					Move(-0.25, 0.25);
					Timer.delay(0.25);
				}
			}
			else if(gameData.charAt(0) == 'R')
			{
				//Right Auto Code 
				if(leftAI.getVoltage() < 4.5)
				{
					Move(0.25, -0.25);
					Timer.delay(40);
					
					Move(1,1);
					Timer.delay(200);
					
					Move(-0.25, 0.25);
					Timer.delay(40);
					
					Move(1,1);
					Timer.delay(200);
					
					Move(-0.25, 0.25);
					Timer.delay(40);				
				}
				else if(rightAI.getVoltage() >= 4.5 && leftAI.getVoltage() >= 4.5)
				{
					Move(0.25, 0.25);
					Timer.delay(1);
					Move(-0.25, 0.25);
					Timer.delay(0.25);
					Move(0.25, 0.25);
					Timer.delay(1);
					Move(0.25, -0.25);
					Timer.delay(0.25);
				}
			}
		}
		boolean running = true;
		double timer = 0;
		led.set(1);		
		
		while(running && isEnabled())
		{
			NetworkTableValue value = vals.getValue();
			if(timer > 6)
			{
				vals.setValue(new String("t"));
				running = false;
			}
			if(value.equals(forward.getValue()))
			{
				Move(-0.5, -0.5);
			}
			else if(value.equals(right.getValue()))
			{
				Move(0.5, -0.5);
			}
			else if(value.equals(left.getValue()))
			{
				Move(-0.5, 0.5);
			}
			
			Timer.delay(0.005);
			timer += 0.005;
		}
		lifter.set(-1);
		Timer.delay(0.5);
		lifter.set(0);
		
		intake.set(-1);
		Timer.delay(1);
		intake.set(0);
				
		led.set(0);
	}

	
	@Override
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
						
			double motorleftvalue = pilotone.getRawAxis(lyaxisone);
			double motorrightvalue = pilotone.getRawAxis(ryaxisone);
			
			boolean turning = false;
			
			if(Math.abs(motorleftvalue) <= deadzone)
			{
				motorleftvalue = 0;
			}
			if(Math.abs(motorrightvalue) <= deadzone)
			{
				motorrightvalue = 0;
			}
			
			if(Math.signum(motorleftvalue) * -1 == Math.signum(motorrightvalue) || !pilotone.getRawButton(ltriggerone))
			{
				motorleftvalue *= 0.6;
				motorrightvalue *= 0.6;
			}
			
			if(pilotone.getRawButton(ltriggerone))
			{
				motorleftvalue *= 0.5;
				motorrightvalue *= 0.5;
			}

			
			/*if(!turning)
			{
				if(motorleftvalue == 0 && prevSpeedL != 0)
				{
					motorleftvalue = prevSpeedL + (Math.signum(prevSpeedL) *  -0.1);
				}
			
				if(motorrightvalue == 0 && prevSpeedR != 0)
				{
					motorrightvalue = prevSpeedR + (Math.signum(prevSpeedR) *  -0.1);
				}
			}*/
			
			
			prevSpeedR = motorrightvalue;
			prevSpeedL = motorleftvalue;
			
			Move(motorleftvalue, motorrightvalue);
			
			double intakeSpeed = pilottwo.getRawAxis(ryaxistwo);
			
			if(Math.abs(intakeSpeed) <= deadzone)
			{
				intakeSpeed = 0;
			}
			
			intake.set(intakeSpeed);
			
			double lifterspeed = pilottwo.getRawAxis(lyaxistwo);
			
			if((Math.abs(lifterspeed) <= deadzone))
			{
				lifterspeed = 0;
			}
			
			/*if((lifterEncoder.getDirection() && lifterspeed < 0) || (!(lifterEncoder.getDirection()) && lifterspeed > 0))
			{
				lifterEncoder.setReverseDirection(true);
			}*/
			
			lifter.set((double)(lifterspeed * -1));
			// The motors will be updated every 5ms
			
			
			Timer.delay(0.005);
		}
	}

	private void Move(double leftvalue, double rightvalue)
	{
		motorleft.set(-1 * leftvalue);
		motorright.set(rightvalue);
	}
	
	//Experimenting with Encoder
	
	/*private void EncoderTesting() {
		Encoder sampleEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		//Max number of seconds before encoder assumes device stops
		sampleEncoder.setMaxPeriod(.1); 
		
		//Minimum rate before encoder assumes device is stopped
		sampleEncoder.setMinRate(10); 
		
		//Sets distance per count on encoder; use rated pulses per revolution stated on encoder
		sampleEncoder.setDistancePerPulse(0.0048); 
		
		//Set direction of encoder
		sampleEncoder.setReverseDirection(true);
		
	}*/
	
}
