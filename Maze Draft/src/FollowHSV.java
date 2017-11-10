import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
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

public class FollowHSV {
	
	static ColorSensor color_sensor = new ColorSensor(SensorPort.S4);
	static EV3TouchSensor touch_sensor = new EV3TouchSensor(SensorPort.S2);
	static EV3IRSensor ir_sensor = new EV3IRSensor(SensorPort.S3);
	
	static EV3LargeRegulatedMotor left_motor = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor right_motor = new EV3LargeRegulatedMotor(MotorPort.A);

	static Wheel wheel1 = WheeledChassis.modelWheel(left_motor , 2.0).offset(-5.5);
	static Wheel wheel2 = WheeledChassis.modelWheel(right_motor , 2.0).offset(5.5);
	static Chassis chassis = new WheeledChassis(new  Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot pilot = new MovePilot(chassis);
	static double[] lineHSV = new double[3];
	static double[] blackHSV= new double[3];
	static double[] woodHSV= new double[3];
	static double [] blueHSV= new double[3];
	static final double TURN_ANGLE = 1;
	static final double SPEED = 6;
	static final double TOLERANCE = 5;
	static final double DISTANCE_FROM_WALL = 21;
	
public static void main(String[] args) throws InterruptedException {
		
		//here we set the speed for the robot before anything else happens
		pilot.setLinearSpeed(SPEED);
		color_sensor.setFloodLight(Color.WHITE);
		Button.waitForAnyPress();
		
		//we assign values to the "places" or colors on the maze. This does not get blue or foil
		calibrateColor(pilot);
		
		Color rgb;
		
		
		
		//this is where we begin motion
		pilot.forward();
		
		while(Button.getButtons() != Button.ID_ESCAPE){ //infinite loop
			
			SensorMode getIR = ir_sensor.getDistanceMode();
			
			//setting the color sensor up to get a new value every time this loops 
			rgb = color_sensor.getColor();
			double[] samplevalue = RGBtoHSV(rgb);
		    System.out.println(samplevalue[0]);
		    
		    if (samplevalue[0]>=190 && samplevalue[0]<=220)//test for blue recognition
		    {
		    		System.out.println("BLUE!!!!!");
		    		//pilot.travel(4);
		    		
		    		if (checkIR(getIR)) {//if we are on the intersection and the IR tells us there is an available left turn
		   				//pilot.travel(5);//travel an arbitrary and untested small amount to get past the wall
		   				//turns.push(directions[0]);//push to the stack before making the turn
		   				pilot.rotate(-50);//take the left turn 
		   				pilot.travel(8);
		   				
		   			}
		   			else {//this is the command to go straight. We will do this every time because it 
		   				pilot.travel(1);//this is just to get past the blue
		   				//turns.push(directions[1]);//push our decision to go straight
		   			}

		    }
		    if ((samplevalue[0] >= (lineHSV[0]-5)) && (samplevalue[0] <= (lineHSV[0]+5))){ //if on the color that is between the line and wood
	    		//it is important to note that the colors, UNLIKE getRedMode() are NOT numerically related to each other.
		    	//We need to say that the robot is ON the wood or ON the line or ON the black to have the proper movements
	    			
		    		//both motors, since we are on the line, are moving forward. This looks like the robot going in a straight line
		    		
		    		left_motor.setSpeed((int)150);
		    		right_motor.setSpeed((int)150);
	    			left_motor.forward();
	    			right_motor.forward();
	    			
	    			
	    			//System.out.println(samplevalue[0]);//print stub
	    			System.out.println("BOUNDARY");
	    			
	    			SensorMode toucher = touch_sensor.getTouchMode(); 
	    			if (checkIfTouching(toucher)) { 		//if robot touches wall w/ touch sensor
	    	   			System.out.println("OUCH !!!");
	    	   			right_motor.stop();
	    	   			left_motor.stop();
	    	   			
	    	   			pilot.travel(-2);
	    	   			pilot.rotate(-160);	//turn around to avoid dead end
	    	   			pilot.travel(6);		//move forward a little so that robot crosses to the correct (right) side of the line
	    	   			left_motor.forward();
	    	   			right_motor.forward();
	    	   		}
	    		}
	    		
	   		else if ((samplevalue[0]>=(89)) && (samplevalue[0] <= (120)) /*|| (samplevalue[0]>=(119)) && (samplevalue[0] <= (121))*/){ //if the color sensor sees the black line (the tolerance is there so we can make better movements), turn right (left motor forward, right motor back)
	   			//NOTE: the tolerance might need to be smaller or larger here. We followed a line through one intersection correctly,
	   			//but got to a less defined one and the robot only went back and forth
	   	
	   			left_motor.forward();//this is to make sure that the left motor is going forward, just in case
	   			left_motor.setSpeed((int)300);//this speed is subject to change. The casting of (int) is to make sure the we are using the correct setSpeed. there is a setSpeed that uses float, but we are using int for consistency 
	   			right_motor.backward();;//this is to set the right motor to move backwards, to make right angles possible
	   			//System.out.println(samplevalue[0]);//print stub	
	   			System.out.println("BLACK");
	    		}
		    
	   		else if ((samplevalue[0]>=(woodHSV[0]-5)) && (samplevalue[0] <= (woodHSV[0]+5))){ //if the color sensor sees the wood (with tolerance), we need to turn left (left motor backwards, right motor forwards) to get back on the line
	   			
	   			right_motor.forward();//making sure the the right motor is still moving forward
	   			right_motor.setSpeed((int)300);//speeding the right motor up to make the turn
	   			left_motor.backward();//setting the left motor to move backwards so we can make right angle turns
	   			//System.out.println(samplevalue[0]);//print stub
	   			System.out.println("WOOD");

	   		}
		    
	   		else {//THIS ELSE STATEMENT IS A CATCH ALL. AS OF NOW, IT IS REACHABLE BUT THAT BEHAVIOR IS NOT WHAT WE NEED. THIS IS LEFT OVER FROM getRedMode() WHERE IT WAS UNREACHABLE, BUT IMPLEMENTED TO PREVENT POSSIBLE BAD BEHAVIOR
	   			left_motor.setSpeed((int)150);
	   			right_motor.setSpeed((int)150);
	   			System.out.println(samplevalue[0]);
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

public static void calibrateColor(MovePilot pilot) throws InterruptedException
{
	color_sensor.setRGBMode();
	color_sensor.setFloodLight(Color.WHITE);//this is for 
	//also we can use the getAmbientLight to get the foil
	Thread.sleep(100);
	
	lineHSV = RGBtoHSV(color_sensor.getColor());
	System.out.println(lineHSV[0]);
	Thread.sleep(100);
	
	pilot.rotate(-30);
	Thread.sleep(100);
	
	blackHSV = RGBtoHSV(color_sensor.getColor());
	System.out.println(blackHSV[0]);
	Thread.sleep(100);
	
	pilot.rotate(60);
	Thread.sleep(100);
	
	woodHSV = RGBtoHSV(color_sensor.getColor());
	System.out.println(woodHSV[0]);
	Thread.sleep(100);
	
	pilot.rotate(-30);
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

public static boolean checkIfTouching(SensorMode sensor){
	float [] samplevalue =  new float [sensor.sampleSize()];
	sensor.fetchSample(samplevalue,0);
	return (samplevalue[0]==1);
}

public static boolean checkIR(SensorMode sensor){
	float [] samplevalue =  new float [sensor.sampleSize()];
	sensor.fetchSample(samplevalue,0);
	return (samplevalue[0]>=DISTANCE_FROM_WALL);//
}

}
