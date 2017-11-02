/** Line Following Program for Lego EV3 Robot
 *  Uses color sensor
 */
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
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
	
	static EV3ColorSensor COLOR_SENSOR = new EV3ColorSensor(SensorPort.S4);
	static EV3TouchSensor TOUCH_SENSOR = new EV3TouchSensor(SensorPort.S2);
	static EV3IRSensor IR_SENSOR = new EV3IRSensor(SensorPort.S3);
	
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	
	static Wheel WHEEL1 = WheeledChassis.modelWheel(LEFT_MOTOR , 2.0).offset(-5.5);
	static Wheel WHEEL2 = WheeledChassis.modelWheel(RIGHT_MOTOR , 2.0).offset(5.5);
	static Chassis chassis = new WheeledChassis(new  Wheel[] { WHEEL1, WHEEL2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot PILOT = new MovePilot(chassis);
	//static double BLACK_COLOR_ID = 0.04;
	//static double FLOOR_COLOR_ID = 0.36;//.37-.35
	//static double BLUE = 0.02; 
	//static double FOIl = 0.86;
	static double TURN_ANGLE = 1;
	static double DISTANCE_FROM_WALL = 21; //this number needs to be measured/changed
	static double SPEED = 6;
	
	//static char[] DIRECTIONS = new char[]{'l','s','r'};
	
	
	public static void main(String[] args) {
		
		Button.waitForAnyPress();
		
		SensorMode getColor1 = COLOR_SENSOR.getRedMode();
		float [] samplevalue1 =  new float [getColor1.sampleSize()];
		getColor1.fetchSample(samplevalue1, 0);
		double boundary = samplevalue1[0];	
		
		Button.waitForAnyPress();
		
		PILOT.setLinearSpeed(SPEED);
		PILOT.setAngularSpeed(PILOT.getMaxAngularSpeed());
		
		//mode that measures reflected light
		
		
		
		/**CALIBRATION
		 * Print out on screen instruction for user to place robot on black line
		 * and press certain button
		 * Read color sensor
		 * Save that condition as 0 value in calibrate mode
		 * Do same thing for the light colored floor, except value is 100
		 */
		
		//the code commented below is intended to deal with calibration of the robot. It still needs to be completed, but this is my first crack at it
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
		
		PILOT.forward();
		
		
		
		while(Button.getButtons() != Button.ID_ESCAPE){  
			
			SensorMode getColor = COLOR_SENSOR.getRedMode();
			float [] samplevalue =  new float [getColor.sampleSize()];
			
		    getColor.fetchSample(samplevalue, 0);
		    System.out.println(samplevalue[0]);
		    
	    		if (samplevalue[0] >(boundary-.02) && samplevalue[0] < (boundary+.02)){ //normal color of floor
	   			
	    			
	    			if(!PILOT.isMoving()) {
	   				PILOT.forward();
	   			}
	   				
	   			//getColor.fetchSample(samplevalue, 0);
	    		}
	    		
	   		else if (samplevalue[0] < (boundary-.02)){ //detects the black line
	   			PILOT.arcForward(1);
	   			if(!PILOT.isMoving()) {
	   				PILOT.forward();
	   			}
	    		}
	   		else if (samplevalue[0] > (boundary+.02)){
	   			PILOT.arcForward(1);
	   			if(!PILOT.isMoving()) {
	   				PILOT.forward();
	   			}
	   		}
	   		else {
	   			if(!PILOT.isMoving()) {
	   				PILOT.forward();
	   			}
	   		}
		}
	}
}
