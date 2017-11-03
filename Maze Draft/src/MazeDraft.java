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
	
	static EV3ColorSensor color_sensor = new EV3ColorSensor(SensorPort.S4);
	static EV3TouchSensor touch_sensor = new EV3TouchSensor(SensorPort.S2);
	static EV3IRSensor ir_sensor = new EV3IRSensor(SensorPort.S3);
	
	static EV3LargeRegulatedMotor left_motor = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor right_motor = new EV3LargeRegulatedMotor(MotorPort.A);
	
	static Wheel wheel1 = WheeledChassis.modelWheel(left_motor , 2.0).offset(-5.5);
	static Wheel wheel2 = WheeledChassis.modelWheel(right_motor , 2.0).offset(5.5);
	static Chassis chassis = new WheeledChassis(new  Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot pilot = new MovePilot(chassis);
	static final double black_color_id = 0.04;
	static final double floor_color_id = 0.36;//.37-.35
	//static final double BLUE = 0.02; 
	//static final double FOIl = 0.86;
	static final double TURN_ANGLE = 1;
	static final double DISTANCE_FROM_WALL = 21; //this number needs to be measured/changed
	static final double SPEED = 6;
	static final double TOLERANCE = .04;
	
	//static final char[] DIRECTIONS = new char[]{'l','s','r'};

	
	public static void main(String[] args) {
		
		pilot.setLinearSpeed(SPEED);
		
		
		Button.waitForAnyPress();
		
		while(Button.getButtons() != Button.ID_ESCAPE){
			//this is where we implement the methods that instigate the actions (also in the methods below)
			
		}
		
		
		
		
		
		//SensorMode getRed = COLOR_SENSOR.getRedMode();
		
		SensorMode getTouch = touch_sensor.getTouchMode();
		
		SensorMode getIR = ir_sensor.getDistanceMode();
		
		pilot.forward();
		
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
				
				pilot.travel(10);
				turnLeft(pilot);
				pilot.travel(5);
				pilot.forward();
				System.out.println("IR");
				
			}
			
			else if(checkIfTouching(getTouch)) {
				backUp(pilot);
				turnRight(pilot);
				pilot.forward();
				System.out.println("TOUCH");
			}
			/*
			else if(!PILOT.isMoving()){
				PILOT.forward();
				System.out.println("NOT MOVING");
			}
			*/
			else {
				//PILOT.forward();
				//System.out.println("ELSE");
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
		return(samplevalue[0] == floor_color_id || samplevalue[0] == black_color_id);
	}
	
	public static void followLine(MovePilot pilot, SensorMode sensor) {
		while(isOffLine(sensor)){
			float [] samplevalue =  new float [sensor.sampleSize()];
			sensor.fetchSample(samplevalue, 0) ;
			
			if(samplevalue[0] == floor_color_id) {
				pilot.rotate(TURN_ANGLE);
			}
			else if(samplevalue[0] == black_color_id) {
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
