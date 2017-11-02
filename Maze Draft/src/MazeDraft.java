//Team10

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

public class MazeDraft {
	static EV3ColorSensor COLOR_SENSOR = new EV3ColorSensor(SensorPort.S4);
	static EV3TouchSensor TOUCH_SENSOR = new EV3TouchSensor(SensorPort.S2);
	static EV3IRSensor IR_SENSOR = new EV3IRSensor(SensorPort.S3);
	
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	
	static Wheel WHEEL1 = WheeledChassis.modelWheel(LEFT_MOTOR , 2.0).offset(-5.5);
	static Wheel WHEEL2 = WheeledChassis.modelWheel(RIGHT_MOTOR , 2.0).offset(5.5);
	static Chassis chassis = new WheeledChassis(new  Wheel[] { WHEEL1, WHEEL2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot PILOT = new MovePilot(chassis);
	static double BLACK_COLOR_ID = 0.04;
	static double FLOOR_COLOR_ID = 0.36;//.37-.35
	static double BLUE = 0.02; 
	static double FOIl = 0.86;
	static double TURN_ANGLE = 2;
	static double DISTANCE_FROM_WALL = 21; //this number needs to be measured/changed
	static double SPEED = 6;
	
	
	public static void main(String[] args) {
		
		PILOT.setLinearSpeed(SPEED);
		
		Button.waitForAnyPress();
		
		while(Button.getButtons() != Button.ID_ESCAPE){
			//this is where we implement the methods that instigate the actions (also in the methods below)
			
		}
		
		
		
		
		
		//SensorMode getRed = COLOR_SENSOR.getRedMode();
		
		SensorMode getTouch = TOUCH_SENSOR.getTouchMode();
		
		SensorMode getIR = IR_SENSOR.getDistanceMode();
		
		PILOT.forward();
		
		while(Button.getButtons() != Button.ID_ESCAPE){ 
			
			/*float [] samplevalue =  new float [getRed.sampleSize()];
			getRed.fetchSample(samplevalue, 0) ;
			
			if(samplevalue[0] == FLOOR_COLOR_ID) {
				PILOT.rotate(TURN_ANGLE);
			}
			else if(samplevalue[0] == BLACK_COLOR_ID) {
				PILOT.rotate(-TURN_ANGLE);
			}
			else
				PILOT.forward();
				*/
			
			if(checkIR(getIR)) {
				
				PILOT.travel(10);
				turnLeft(PILOT);
				PILOT.travel(5);
				PILOT.forward();
				
			}
			else if(checkIfTouching(getTouch)) {
				backUp(PILOT);
				turnRight(PILOT);
				PILOT.forward();
			}
			
		}
		
		
	}
	
	public static boolean checkIR(SensorMode sensor){
		float [] samplevalue =  new float [sensor.sampleSize()];
		sensor.fetchSample(samplevalue,0);
		return (samplevalue[0]>=DISTANCE_FROM_WALL);//
	}
	
	public static boolean checkIfTouching(SensorMode sensor){
		float [] samplevalue =  new float [sensor.sampleSize()];
		sensor.fetchSample(samplevalue,0);
		return (samplevalue[0]==1);
	}
	
	/*
	public static boolean checkColor(SensorMode sensor){
		//To tell the difference between foil and blue, checkColor() uses getRedMode() to give different readings
		//returns true if the color is blue, false if it is the foil
		boolean isBlue=false;
		return isBlue;
	}
	*/
	
	public static boolean isOffLine(SensorMode sensor) {
		float [] samplevalue =  new float [sensor.sampleSize()];
		sensor.fetchSample(samplevalue, 0) ;
		return(samplevalue[0] == FLOOR_COLOR_ID || samplevalue[0] == BLACK_COLOR_ID);
	}
	
	public static void followLine(MovePilot pilot, SensorMode sensor) {
		while(isOffLine(sensor)){
			float [] samplevalue =  new float [sensor.sampleSize()];
			sensor.fetchSample(samplevalue, 0) ;
			
			if(samplevalue[0] == FLOOR_COLOR_ID) {
				pilot.rotate(TURN_ANGLE);
			}
			else if(samplevalue[0] == BLACK_COLOR_ID) {
				pilot.rotate(-TURN_ANGLE);
			}
		}
	}
	
	public static void turnRight(MovePilot pilot){
		pilot.stop();
	    	pilot.rotate(90);
	    	pilot.forward();
	}
	
	public static void turnLeft(MovePilot pilot){
		pilot.stop();
	    	pilot.rotate(-90);
	    	pilot.forward();
	}
	
	public static void backUp(MovePilot pilot){
		pilot.travel(-2);
	}
		
}
