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

public class MazeSolver {
	
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
	static final int[] directions = {0, 1, 2};
	static int lastIntersection =-1;
	
	public static void main(String[] args) throws InterruptedException {
		
		boolean end = false;
		
		Stack<Integer> turns = new Stack<Integer>();
	
		//here we set the speed for the robot before anything else happens
		pilot.setLinearSpeed(SPEED);
		color_sensor.setFloodLight(Color.WHITE);
		Button.waitForAnyPress();
		
		//we assign values to the "places" or colors on the maze. This does not get blue or foil
		//calibrateColor(pilot);
		color_sensor.setRGBMode();
		
		Color rgb;
		
		
		
		//this is where we begin motion
		pilot.forward();
		
		//this loop goes until we reach the foil
		while(!end){ 
			
			SensorMode getIR = ir_sensor.getDistanceMode();
			
			//setting the color sensor up to get a new value every time this loops 
			rgb = color_sensor.getColor();
			double[] samplevalue = RGBtoHSV(rgb);
		    System.out.println(samplevalue[0]);
		    
		    if (samplevalue[0]>=190 && samplevalue[0]<=220)//test for blue recognition
		    {
		    		System.out.println("BLUE!!!!!");
		    		//pilot.travel(4);
		    		
		    		//if last intersection has an int value between 0-2 inclusive, then we MUST be coming back from a dead end, and our turn decisions ONLY depend on that
		   		if(lastIntersection >= 0 && lastIntersection <= 2) {
		   				
		   				//if we are at the intersection AND the last intersection we went left, then now we must go left.
		   				//We ALSO need to push straight, which is what the robot will do on the way back
		   			if (lastIntersection==0) {
		   				//correct the stack to what we should have done here
		   				turns.push(1);
		   				// this resets lastIntersection to not be out of bounds, so we do not accidentally think we need to correct a mistake again
		   				lastIntersection =-1; 
		   				//make the turn
		   				pilot.rotate(-55);//take the left turn 
		   				pilot.travel(7);
		   			}
		   			
		   			//same idea, if we went straight and we got a dead end, then we MUST go left to go the other way
	   				//(this is because we could not have gone left, meaning that the only options were straight and right)
	   				else if (lastIntersection == 1) {
	   					
	   					//this is what we should have done here. This assumes that left was not an option and we made an incorrect decision to go straight.
	   					//This corrects so the stack will have correct instructions later
	   					turns.push(2);
	   					// this resets lastIntersection to not be out of bounds, so we do not accidentally think we need to correct a mistake again
	   					lastIntersection =-1;
	   					//make the correct turn
	   					pilot.rotate(-55);//take the left turn 
		   				pilot.travel(7);
	   				}
		   		}
		   		//if we are on the intersection and the IR tells us there is an available left turn
		    		if (checkIR(getIR)) {
		    				//push to the stack before making the turn
		   				turns.push(directions[0]);
		   				//take the left turn
		   				pilot.rotate(-55); 
		   				pilot.travel(7);
		   				
		   			}
		    			//this is the command to go straight. We will do this every time because it 
		   			else {
		   				left_motor.forward();
		   				right_motor.forward();
		   				left_motor.setSpeed((int)100);
		   				right_motor.setSpeed((int)100);
		   				//push our decision to go straight
		   				turns.push(directions[1]);
		   				
		   				//this is just to get past the blue
		   				pilot.travel(6);
		   				
		   			}

		    }
		    else if ((samplevalue[0] >= 39) && (samplevalue[0] <= 43)){ //if on the color that is between the line and wood
	    		//it is important to note that the colors, UNLIKE getRedMode() are NOT numerically related to each other.
		    	//We need to say that the robot is ON the wood or ON the line or ON the black to have the proper movements
	    			
		    		//both motors, since we are on the line, are moving forward. This looks like the robot going in a straight line
		    		
		    		left_motor.setSpeed((int)100);
		    		right_motor.setSpeed((int)100);
	    			left_motor.forward();
	    			right_motor.forward();
	    			
	    			
	    			//System.out.println(samplevalue[0]);//print stub
	    			System.out.println("BOUNDARY");
	    			
	    			SensorMode toucher = touch_sensor.getTouchMode(); 
	    			//if robot touches wall w/ touch sensor
	    			if (checkIfTouching(toucher)) { 		
	    	   			
	    				System.out.println("OUCH !!!");
	    	   			right_motor.stop();
	    	   			left_motor.stop();
	    	   			lastIntersection = turns.pop();
	    	   			
	    	   			
	    	   			pilot.travel(-2);
	    	   			//turn around to avoid dead end
	    	   			pilot.rotate(-160);
	    	   			//move forward a little so that robot crosses to the correct (right) side of the line
	    	   			pilot.travel(6);		
	    	   			left_motor.forward();
	    	   			right_motor.forward();
	    	   		}
	    			
	    		}
	    		
		    
		    //if the color sensor sees the black line (the tolerance is there so we can make better movements), turn right (left motor forward, right motor back)
	   		else if ((samplevalue[0]>=(89)) && (samplevalue[0] <= (120))){ 
	   			
	   			//this is to make sure that the left motor is going forward, just in case
	   			left_motor.forward();
	   			//this speed is subject to change. The casting of (int) is to make sure the we are using the correct setSpeed. there is a setSpeed that uses float, but we are using int for consistency 
	   			left_motor.setSpeed((int)200);
	   			//this is to set the right motor to move backwards, to make right angles possible
	   			right_motor.backward();
	   			//System.out.println(samplevalue[0]);//print stub	
	   			System.out.println("BLACK");
	   			Thread.sleep(100);
	    		}
		    
	   		else if ((samplevalue[0]>=29) && (samplevalue[0] <= 33)){ //if the color sensor sees the wood (with tolerance), we need to turn left (left motor backwards, right motor forwards) to get back on the line
	   			
	   			Thread.sleep(50);
	   			//making sure the the right motor is still moving forward
	   			right_motor.forward();
	   			//speeding the right motor up to make the turn
	   			right_motor.setSpeed((int)200);
	   			//setting the left motor to move backwards so we can make right angle turns
	   			left_motor.backward();
	   			//System.out.println(samplevalue[0]); //print stub
	   			System.out.println("WOOD");
	   			Thread.sleep(100);

	   		}
	   		else if ((samplevalue[0]>=85) && (samplevalue[0]<=87)){
	   			
	   			System.out.println("I REACHED THE FOIL");
	   			System.out.println(samplevalue[0]);
	   			pilot.stop();
	   			pilot.rotate(-170);
	   			pilot.travel((int)6);
	   			end = true;
	   			Thread.sleep(1000);
	   			
	   		}
		    
	   		else {
	   			left_motor.setSpeed((int)100);
	   			right_motor.setSpeed((int)100);
	   			System.out.println(samplevalue[0]);
	   		}
		}
		
		while(end) {
			
			rgb = color_sensor.getColor();
			double[] samplevalue = RGBtoHSV(rgb);
			
			//if we are at an intersection, the robot now only looks at the stack to determine which direction to turn
			if (samplevalue[0]>=190 && samplevalue[0]<=220) { 
				
				//now we use lastIntersection as the "map" back
				lastIntersection = turns.pop();
				
				//if the map said we went left here, we need to go right on the way back
				if(lastIntersection == 0) {
					//inverse of a correct left turn would be a right turn
					//take the right turn 
					pilot.rotate(100);
		   			pilot.travel(3);
				}
				//if the map says that we went straight, we can just go straight still
				else if(lastIntersection == 1) {
					//this is go to straight and skip the blue square
					pilot.travel(5);
				}
				//if the map says that we went right, we need to go left instead
				else if (lastIntersection == 2) {
					//the inverse of a correct right turn is a left turn
					//take the left turn 
					pilot.rotate(-55);
	   				pilot.travel(7);
				}
			}
			else if ((samplevalue[0] >= 39) && (samplevalue[0] <= 43)){ //if on the color that is between the line and wood
	    		//it is important to note that the colors, UNLIKE getRedMode() are NOT numerically related to each other.
		    	//We need to say that the robot is ON the wood or ON the line or ON the black to have the proper movements
	    			
		    		//both motors, since we are on the line, are moving forward. This looks like the robot going in a straight line
		    		
		    		left_motor.setSpeed((int)100);
		    		right_motor.setSpeed((int)100);
	    			left_motor.forward();
	    			right_motor.forward();
	    			Thread.sleep(100);
	    			
	    			//System.out.println(samplevalue[0]);//print stub
	    			System.out.println("BOUNDARY");
	    			
	    			SensorMode toucher = touch_sensor.getTouchMode(); 
	    			if (checkIfTouching(toucher)) { 		//if robot touches wall w/ touch sensor
	    	   			System.out.println("THE MAZE HAS BEN SOLVEDTH!!!!!!!!!");
	    	   			pilot.stop();
	    	   			Thread.sleep(10000);
	    	   		}
	    			
	    		}
	    		
			//if the color sensor sees the black line (the tolerance is there so we can make better movements), turn right (left motor forward, right motor back)
	   		else if ((samplevalue[0]>=(89)) && (samplevalue[0] <= (120)) ){ 
	   			
	   			
	   			//this is to make sure that the left motor is going forward, just in case
	   			left_motor.forward();
	   			//this speed is subject to change. The casting of (int) is to make sure the we are using the correct setSpeed. there is a setSpeed that uses float, but we are using int for consistency
	   			left_motor.setSpeed((int)200); 
	   			//this is to set the right motor to move backwards, to make right angles possible
	   			right_motor.backward();
	   			//System.out.println(samplevalue[0]);//print stub	
	   			System.out.println("BLACK");
	   			Thread.sleep(100);
	    		}
		    
			//if the color sensor sees the wood (with tolerance), we need to turn left (left motor backwards, right motor forwards) to get back on the line
	   		else if ((samplevalue[0]>=29) && (samplevalue[0] <= 32)){ 
	   			
	   			Thread.sleep(50);
	   			//making sure the the right motor is still moving forward
	   			right_motor.forward();
	   			//speeding the right motor up to make the turn
	   			right_motor.setSpeed((int)200);
	   			//setting the left motor to move backwards so we can make right angle turns
	   			left_motor.backward();
	   			//System.out.println(samplevalue[0]);//print stub
	   			System.out.println("WOOD");
	   			Thread.sleep(100);

	   		}
		    
	   		else {
	   			left_motor.setSpeed((int)100);
	   			right_motor.setSpeed((int)100);
	   			System.out.println(samplevalue[0]);
	   		}
		}
	
	}		
	

	public static boolean checkIR(SensorMode sensor){
		float [] samplevalue =  new float [sensor.sampleSize()];
		sensor.fetchSample(samplevalue,0);
		return (samplevalue[0]>=DISTANCE_FROM_WALL);
	}
	
	public static boolean checkIfTouching(SensorMode sensor){
		float [] samplevalue =  new float [sensor.sampleSize()];
		sensor.fetchSample(samplevalue,0);
		return (samplevalue[0]==1);
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
