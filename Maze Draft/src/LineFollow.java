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
	
	static EV3ColorSensor color_sensor = new EV3ColorSensor(SensorPort.S4);
	static EV3TouchSensor touch_sensor = new EV3TouchSensor(SensorPort.S2);
	static EV3IRSensor ir_sensor = new EV3IRSensor(SensorPort.S3);
	
	static EV3LargeRegulatedMotor left_motor = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor right_motor = new EV3LargeRegulatedMotor(MotorPort.A);
	
	static Wheel wheel1 = WheeledChassis.modelWheel(left_motor , 2.0).offset(-5.5);
	static Wheel wheel2 = WheeledChassis.modelWheel(right_motor , 2.0).offset(5.5);
	static Chassis chassis = new WheeledChassis(new  Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot pilot = new MovePilot(chassis);
	//static final double BLACK_COLOR_ID = 0.04;
	//static final double FLOOR_COLOR_ID = 0.36;//.37-.35
	//static final double BLUE = 0.02; 
	//static final double FOIl = 0.86;
	static final double TURN_ANGLE = 1;
	static final double DISTANCE_FROM_WALL = 21; //this number needs to be measured/changed
	static final double SPEED = 6;
	static final double TOLERANCE = 0.04;
	
	//static final char[] DIRECTIONS = new char[]{'l','s','r'};
	
	
	public static void main(String[] args) throws InterruptedException {
		
		Button.waitForAnyPress();
		
		SensorMode getColor1 = color_sensor.getRedMode();
		float [] samplevalue1 =  new float [getColor1.sampleSize()];
		getColor1.fetchSample(samplevalue1, 0);
		
		double boundary = samplevalue1[0];	
		
		//Button.waitForAnyPress();
		
		pilot.setLinearSpeed(SPEED);
		pilot.setAngularAcceleration(720);
		//pilot.setAngularSpeed(pilot.getMaxAngularSpeed());
		
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
		
		pilot.forward();
		
		while(Button.getButtons() != Button.ID_ESCAPE){  
			
			SensorMode getColor = color_sensor.getRedMode();
			float [] samplevalue =  new float [getColor.sampleSize()];
			
		    getColor.fetchSample(samplevalue, 0);
		    System.out.println(samplevalue[0]);
		    
	    		if ((samplevalue[0] >= (boundary-TOLERANCE)) && (samplevalue[0] <= (boundary+TOLERANCE))){ //normal color of floor
	    			
	    			
	    			left_motor.forward();
	    			right_motor.forward();
	    			left_motor.setSpeed((int)90);
	    			right_motor.setSpeed((int)90);
	    			
	    			//pilot.forward();
	    			
	    			System.out.println(samplevalue[0]);
	    			
	    			/*
	    			if(!pilot.isMoving())
	    				pilot.forward();
	   			*/
	   				
	   			//getColor.fetchSample(samplevalue, 0);
	    		}
	    		
	   		else if (samplevalue[0] < (boundary -TOLERANCE)){ 
	   		
	   			//while(samplevalue[0] < (boundary -TOLERANCE)){
	   			left_motor.forward();
	   			left_motor.setSpeed((int)200);
	   			right_motor.backward();
	   			//getColor.fetchSample(samplevalue, 0);
	   			//}
	   			//pilot.arcForward(-5);
	   			
	   			//Thread.sleep(100);
				
	   			
	   			//PILOT.forward();
	   			System.out.println(samplevalue[0]);
	   			
	   			
	   			
	    		}
	   		else if (samplevalue[0] > (boundary + TOLERANCE)){
	   			
	   			//while (samplevalue[0] > (boundary + TOLERANCE)){
	   			right_motor.forward();
	   			right_motor.setSpeed((int)200);
	   			left_motor.backward();
	   			//getColor.fetchSample(samplevalue, 0);
	   			//}
	   			//pilot.arcForward(5);
	   			
	   			//Thread.sleep(100);
	   			//pilot.arcForward(0.5);
	   			
	   			//PILOT.forward();
	   			System.out.println(samplevalue[0]);
	   		}
	   		//else if(samplevalue[0] == BLUE)
	    		
	   		else {
	   			
	   			left_motor.setSpeed((int)90);
	   			right_motor.setSpeed((int)90);
	   			
	   			System.out.println(samplevalue[0]);
	   			
	   			/*if(!pilot.isMoving()) {
	   				pilot.forward();
	   			}*/
	   		}
		}
	}
}
