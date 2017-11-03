import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.utility.Delay;

public class RGBtoHSVDemo {
	
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

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		ColorSensor colorSensor = new ColorSensor(SensorPort.S4);
		colorSensor.setRGBMode();
		Color rgb; 
		
		
		Button.waitForAnyPress(); 
		while(Button.ESCAPE.isUp())
		{
			rgb = colorSensor.getColor();
			double[] hsv = RGBtoHSV(rgb);
            System.out.println("RGB = "+
                " [" + rgb.getRed() + "," + rgb.getGreen() + "," + rgb.getBlue() 
                +"]\n "  + "HSV = " + "[ " + hsv[0] + "," + hsv[1] + "," + hsv[2] + "," +" ]");
			//delay a second so we don't see so many outputs on the screen
			Delay.msDelay(5000);	
		}
			
		
		//free up resources
		colorSensor.close();
		
	}

}
