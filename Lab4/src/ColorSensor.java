import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

/** Color Sensor navigation Program for Lego EV3 Robot
 * 
 *  Uses color sensor to go until color a color (red or blue) is detected,
 *  then turns 90 degrees and goes until the second color is detected
 *  
 */

public class ColorSensor {

	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	
	static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.C);
	static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	
	static Wheel wheel1 = WheeledChassis.modelWheel(leftMotor , 3.0).offset(-5.5);
	static Wheel wheel2 = WheeledChassis.modelWheel(rightMotor , 3.0).offset(5.5);
	static Chassis chassis = new WheeledChassis(new  Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot pilot = new MovePilot(chassis);
	
	public static void main(String[] args) {
		Button.waitForAnyPress();
		
		//mode that measures reflected light
		int getColor = colorSensor.getColorID();
		
		int firstColor;
		
		while (Button.getButtons() != Button.ID_ESCAPE) {
			
			pilot.forward();
			
			if(getColor == Color.RED || getColor == Color.BLUE) {
				firstColor = getColor;
				pilot.stop();
				pilot.rotate(90);
				pilot.forward();
				
				//if(getColor == Opposite of getColor)
					//pilot.stop();
			}
		}
	}
	
}
