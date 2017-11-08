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
	static final double TOLERANCE = .04;
	
public static void main(String[] args) throws InterruptedException {
		
		pilot.setLinearSpeed(SPEED);
		Button.waitForAnyPress();
		
		calibrateColor(pilot);
		
		Color rgb;
		
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
	    		}
	    		
	   		else if (samplevalue[0] == blackHSV[0]){ 
	   	
	   			left_motor.forward();
	   			left_motor.setSpeed((int)200);
	   			right_motor.backward();
	   			System.out.println(samplevalue[0]);	
	    		}	   		
	   		else {
	   			left_motor.setSpeed((int)90);
	   			right_motor.setSpeed((int)90);
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

public static boolean checkIfTouching(SensorMode sensor){
	float [] samplevalue =  new float [sensor.sampleSize()];
	sensor.fetchSample(samplevalue,0);
	return (samplevalue[0]==1);
}

}
