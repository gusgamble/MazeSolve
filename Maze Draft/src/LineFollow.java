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
		
		
		pilot.setLinearSpeed(SPEED);		
		pilot.forward();
		SensorMode toucher = touch_sensor.getTouchMode();
		
		while(Button.getButtons() != Button.ID_ESCAPE){  
			
			SensorMode getColor = color_sensor.getRedMode();
			float [] samplevalue =  new float [getColor.sampleSize()];
			
		    getColor.fetchSample(samplevalue, 0);
		    System.out.println(samplevalue[0]);
		    
	    		if ((samplevalue[0] >= (boundary-TOLERANCE)) && (samplevalue[0] <= (boundary+TOLERANCE))){ //normal color of floor
	    			
	    			
	    			left_motor.forward();
	    			right_motor.forward();
	    			left_motor.setSpeed((int)150);
	    			right_motor.setSpeed((int)150);
	    			
	    			
	    			
	    			System.out.println(samplevalue[0]);
	    		}
	    		
	   		else if (samplevalue[0] < (boundary -TOLERANCE)){ 
	   	
	   			left_motor.forward();
	   			left_motor.setSpeed((int)300);
	   			right_motor.backward();
	   			System.out.println(samplevalue[0]);
	   			
	   			
	   			
	    		}
	   		else if (samplevalue[0] > (boundary + TOLERANCE)){
	   			
	   			right_motor.forward();
	   			right_motor.setSpeed((int)300);
	   			left_motor.backward();
	   			System.out.println(samplevalue[0]);
	   		}
	    		
	   		else if (checkIfTouching(toucher)) {
	   			pilot.travel(-2);
	   			pilot.rotate(180);
	   		}
	    		
	   		else {
	   			
	   			left_motor.setSpeed((int)150);
	   			right_motor.setSpeed((int)150);
	   			
	   			System.out.println(samplevalue[0]);
	   			
	   		}
		}
	}
	
	public static boolean checkIfTouching(SensorMode sensor){
		float [] samplevalue =  new float [sensor.sampleSize()];
		sensor.fetchSample(samplevalue,0);
		return (samplevalue[0]==1);
	}
}
