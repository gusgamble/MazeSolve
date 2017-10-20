/** Line Following Program for Lego EV3 Robot
 *  Uses color sensor
 */
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.robotics.navigation.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor; 
import lejos.hardware.port.MotorPort; 
import lejos.robotics.chassis.Chassis; 
import lejos.robotics.chassis.Wheel; 
import lejos.robotics.chassis.WheeledChassis;

public class LineFollow {
	
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	
	static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.C);
	static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	
	static Wheel wheel1 = WheeledChassis.modelWheel(leftMotor , 3.0).offset(-5.5);
	static Wheel wheel2 = WheeledChassis.modelWheel(rightMotor , 3.0).offset(5.5);
	static Chassis chassis = new WheeledChassis(new  Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot pilot = new MovePilot(chassis);
	static double blackColorID = 7.0; //Hardcoded for now. Will change once calibration is debugged.
	static double whiteColorID = 2.0; //Hardcoded for now. Will change once calibration is debugged.
	
	public static void main(String[] args) {
		Button.waitForAnyPress();
		
		//mode that measures reflected light
		SensorMode getColor = colorSensor.getColorIDMode();
		float [] samplevalue =  new float [getColor.sampleSize()];
		
		/**CALIBRATION
		 * Print out on screen instruction for user to place robot on black line
		 * and press certain button
		 * Read color sensor
		 * Save that condition as 0 value in calibrate mode
		 * Do same thing for the light colored floor, except value is 100
		 */
		
		//the code commented below is intended to deal with calibration of the robot. It still needs to be completed, but this is my first crack at it -gus
		/*
		System.out.println("Place the robot on the line and then press the x button");
		
		int b1 = Button.waitForAnyPress();
		if (b1 == x) {
			float blackLine = getColor.sampleSize();
		}
		
		System.out.println("Place the robot on the ground and then press the y button");
		
		int b2 = Button.waitForAnyPress(); 
		if (b2 == y) {
			float floor = getColor.sampleSize();
		}*/
		
		pilot.forward();
		
		while(Button.getButtons() != Button.ID_ESCAPE){                   
		    getColor.fetchSample(samplevalue, 0);
		    System.out.println(samplevalue[0]);
		    
	    		if (samplevalue[0] == whiteColorID){ //normal color of floor
	    			
	   			pilot.rotate(-15); 
	   			getColor.fetchSample(samplevalue, 0);
	    		}
	    		
	   		else if (samplevalue[0] == blackColorID){ //detects the black line
	   			
	   			pilot.rotate(15);
	   			getColor.fetchSample(samplevalue, 0);
	    		}
	    		
	   		else if (samplevalue[0]<whiteColorID && samplevalue[0]>blackColorID) {
	   			
	   			pilot.forward();
	   			getColor.fetchSample(samplevalue, 0);
	   			
	   		}
		}
	}
}
