//Team10

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;

import java.util.Stack;

import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.robotics.navigation.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor; 
import lejos.hardware.port.MotorPort;
import lejos.robotics.Color;
import lejos.robotics.chassis.Chassis; 
import lejos.robotics.chassis.Wheel; 
import lejos.robotics.chassis.WheeledChassis;
import lejos.utility.Delay;

public class MazeDraft {
	
	static ColorSensor color_sensor = new ColorSensor(SensorPort.S4);
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
	static double[] lineHSV = new double[3];
	static double[] blackHSV= new double[3];
	static double[] woodHSV= new double[3];
	static double [] blueHSV= new double[3];
	//static final double BLUE = 0.02; 
	//static final double FOIl = 0.86;
	static final double TURN_ANGLE = 1;
	static final double DISTANCE_FROM_WALL = 21; //this number needs to be measured/changed
	static final double SPEED = 6;
	static final double TOLERANCE = .04;
	static int[] directions = {0, 1, 2};
	static int lastIntersection =4;
	
	//static final char[] DIRECTIONS = new char[]{'l','s','r'};

	
	public static void main(String[] args) throws InterruptedException {
		
		Stack<Integer> turns = new Stack<Integer>();
		
		pilot.setLinearSpeed(SPEED);
		Button.waitForAnyPress();
		
		calibrateColor(pilot);
		Thread.sleep(1000000000);
		
		Color rgb;
		
		SensorMode getTouch = touch_sensor.getTouchMode();
		
		SensorMode getIR = ir_sensor.getDistanceMode();
		
		pilot.forward();
		
		//double boundary = 5;
		
		while(Button.getButtons() != Button.ID_ESCAPE){ 
			
			
			rgb = color_sensor.getColor();
			double[] samplevalue = RGBtoHSV(rgb);
		    System.out.println(samplevalue[0]);
		    
	    		if ((samplevalue[0] == woodHSV[0])){ //normal color of floor
	    			
	    			
	    			left_motor.forward();
	    			right_motor.forward();
	    			left_motor.setSpeed((int)90);
	    			right_motor.setSpeed((int)90);
	    			
	    			System.out.println(samplevalue[0]);
	    			
	    			if (checkIfTouching(getTouch)) {
	    				lastIntersection = turns.pop();
	    			}
	    		}
	    		
	   		else if (samplevalue[0] == blackHSV[0]){ 
	   	
	   			left_motor.forward();
	   			left_motor.setSpeed((int)200);
	   			right_motor.backward();
	   			System.out.println(samplevalue[0]);
	   			
	   			
	   			
	    		}
	   		else if (samplevalue[0] == woodHSV[0]){
	   			
	   			right_motor.forward();
	   			right_motor.setSpeed((int)200);
	   			left_motor.backward();
	   			System.out.println(samplevalue[0]);
	   		}
	    		//ABOVE THIS COMMENT, SEE COMMENTS IN FollowHSV FOR THE LOGIC THAT PERTAINS TO HSV IN LINE FOLLOWING
	    		
	   		else if(samplevalue[0] == blueHSV[0]) {//this is the code that should deal with when we hit an intersection
	   			if(lastIntersection < 0||lastIntersection>2) {//if last intersection has an int value between 0-2 inclusive, then we MUST be coming back from a dead end here
	   				if (lastIntersection==0) {//if we are at the intersection AND the last intersection we went left, then now we must go left. We ALSO need to push straight, which is what the robot will do on the way back
	   					turns.push(1);//correct the stack to what we should have done here
	   					lastIntersection =-1; // this resets lastIntersection to not be out of bounds, so we do not accidentally think we need to correct a mistake again
	   					turnLeft(pilot);//make the turn
	   				}
	   				else if (lastIntersection == 1) {//same idea, if we went straight and we got a dead end, then we MUST go left to go the other way (this is because we could not have gone left, meaning that the only options were straight and right)
	   					turns.push(2);//this is what we should have done here. This assumes that left was not an option and we made an incorrect decision to go straight. This corrects so the stack will have correct instructions later
	   					lastIntersection =-1; // this resets lastIntersection to not be out of bounds, so we do not accidentally think we need to correct a mistake again
	   					turnLeft(pilot);//make the correct turn
	   				}

	   			}
	   			else if (checkIR(getIR)) {//if we are on the intersection and the IR tells us there is an available left turn
	   				pilot.travel(5);//travel an arbitrary and untested small amount to get past the wall
	   				turns.push(directions[0]);//push to the stack before making the turn
	   				turnLeft(pilot);//take the left turn 
	   				
	   			}
	   			else {//this is the command to go straight. We will do this every time because it 
	   				pilot.travel(5);//this is just to get past the blue
	   				turns.push(directions[1]);//push our decision to go straight
	   			}
	   		}
	    		
	   		else if (checkIfTouching(getTouch)) {//this checks if we have hit a dead end
	   			lastIntersection = turns.pop();//give the most recent decision, which must have been wrong, to the variable that can store it and use it to make a correction when back at the intersection
	   			pilot.travel(-2);//back up 
	   			pilot.rotate(180);//turn around
	   			//HERE IS WHERE WE WILL MERGE THE LOGIC FROM MELODY'S CODE IN LINEFOLLOW
	   		}
	    		
	   		else {//this is just a safety net to catch every possible if
	   			
	   			left_motor.setSpeed((int)90);
	   			right_motor.setSpeed((int)90);
	   			
	   			System.out.println(samplevalue[0]);
	   			
	   		}
			
		}
		boolean end = false;//this will be declared at the beginning of the program and will only be set to true when the robot has reached the foil
		//the foil behavior is pilot.stop(), play noise, end=true, and finally break
		while(end) {
			
			
			
			//line following code goes here. the if statements below belong in the if case of all(?) of the statements
			
			rgb = color_sensor.getColor();
			double[] samplevalue = RGBtoHSV(rgb);
			
			if (samplevalue[0]== blueHSV[0] ) {//if we are at an intersection, the robot now only looks at the stack to determine how 
				lastIntersection = turns.pop();//now we use lastIntersection as the "map" back
				
				if(lastIntersection == 0) {//if the map said we went left here, we need to go right on the way back
					turnRight(pilot);//inverse of a correct left turn would be a right turn
				}
				else if(lastIntersection == 1) {//if the map says that we went straight, we can just go straight still
					pilot.travel(5);//this is go to straight and skip the blue square
				}
				else if (lastIntersection == 2) {//if the map says that we went right, we need to go left instead
					turnLeft(pilot);//the inverse of a correct right turn is a left turn
				}
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

	
	// THIS IS WHERE THE CODE FOR THE LINE FOLLOW GOES
	public static boolean isOffLine(SensorMode sensor) {
		float [] samplevalue =  new float [sensor.sampleSize()];
		sensor.fetchSample(samplevalue, 0) ;
		return(samplevalue[0] == floor_color_id || samplevalue[0] == black_color_id);
	}
	/*
	public static void followLine(MovePilot pilot, SensorMode sensor, double boundary) {
		
		
			
			//the error below will be fixed when hsv is implemented
			sensor = color_sensor.getRedMode();
			float [] samplevalue =  new float [sensor.sampleSize()];
			
		    sensor.fetchSample(samplevalue, 0);
		    System.out.println(samplevalue[0]);
		    
	    		if ((samplevalue[0] >= (boundary-TOLERANCE)) && (samplevalue[0] <= (boundary+TOLERANCE))){ //normal color of floor
	    			
	    			
	    			left_motor.forward();
	    			right_motor.forward();
	    			left_motor.setSpeed((int)90);
	    			right_motor.setSpeed((int)90);
	    			
	    			
	    			
	    			System.out.println(samplevalue[0]);
	    		}
	    		
	   		else if (samplevalue[0] < (boundary -TOLERANCE)){ 
	   	
	   			left_motor.forward();
	   			left_motor.setSpeed((int)200);
	   			right_motor.backward();
	   			System.out.println(samplevalue[0]);
	   			
	   			
	   			
	    		}
	   		else if (samplevalue[0] > (boundary + TOLERANCE)){
	   			
	   			right_motor.forward();
	   			right_motor.setSpeed((int)200);
	   			left_motor.backward();
	   			System.out.println(samplevalue[0]);
	   		}
	    		
	   		else {
	   			
	   			left_motor.setSpeed((int)90);
	   			right_motor.setSpeed((int)90);
	   			
	   			System.out.println(samplevalue[0]);
	   			
	   		}
		
	}
	*/
	
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

	public static void calibrateColor(MovePilot pilot) throws InterruptedException
	{
		color_sensor.setRGBMode();
		
		lineHSV = RGBtoHSV(color_sensor.getColor());
		System.out.println(lineHSV[0]);
		Thread.sleep(500);
		
		pilot.rotate(-20);
		Thread.sleep(100);
		
		blackHSV = RGBtoHSV(color_sensor.getColor());
		System.out.println(blackHSV[0]);
		Thread.sleep(500);
		
		pilot.rotate(40);
		Thread.sleep(100);
		
		woodHSV = RGBtoHSV(color_sensor.getColor());
		System.out.println(woodHSV[0]);
		Thread.sleep(500);
		
		pilot.rotate(-20);
	}
	
	public static double[] RGBtoHSV(Color colors){
		double[] hsv = new double[3];
		// read colors
		int r = colors.getRed();
		int b = colors.getBlue();
		int g = colors.getGreen();
		
		double min = Math.min(r, Math.min(b,g));
		double max = Math.max(r, Math.max(b, g));
		double delta = max - min;
		hsv[2] = max/255; //set v to max as a percentage
		if (max != 0){
			hsv[1] = delta/max;
		}
		else{ //r = b = g =0 
			hsv[1] = 0; //s = 0;		// s = 0, v is undefined
			hsv[0] = -1; //h = -1;
			return hsv;
		}
		
		if (r == max){
			hsv[0] = (g-b)/delta; //h 
		}
		else{
			if (g == max)
				hsv[0] = 2 + (b - r)/delta; //h
			else
				hsv[0] = 4 + (r - g)/delta; //h
		}
		
		hsv[0] *=60;	//degrees
		if (hsv[0] < 0)
			hsv[0] +=360;
		
		return hsv;
	}
	
}
