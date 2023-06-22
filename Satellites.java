
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Scanner;

/**
 * The role of satellites is to send information to the receiver about where satellites are 
 * and what time they sent the information. The receiver uses the time difference between 
 * the signal being sent and being received to calculate its distance from each satellite. 
 * Since this is only a simulation, this class is given information about the vehicle's 
 * position at a given time. It then works backwards to calculate the time it would take
 * a signal to travel from each satellite above the horizon to the vehicle. This program 
 * sends the receiver the position of each satellite that is above the horizon and the time
 * at which that satellite would have had to have sent the signal for it to reach the vehicle
 * at the desired time in the desired position. 
 */
public class Satellites {
	//pi, speed of light, radius of the earth
	private static double pi, c, R;
	//time the vehicle should recieve the signals
	private double time;
	//cartesian coordinates of the vehicle's location
	private double[] vehiclePosition;
	//list of 24 satellites
	private Satellite[] satList;


	/**
	 * Satellite objects hold information about their movement and the catresian coordinates of their
	 * position at desired time
	 */
	private class Satellite{
		private double u1, u2, u3, v1, v2, v3, period, altitude, phase;
		private double[] cartesianCoordinates;

		/**
		 * To initialize a satellite object, specifiy it's positional data (unit vector, orthoganol 
		 * vector, period, altitude, phase)
		 * @param position
		 */
		private Satellite(double[] position){
			u1 = position[0];
			u2 = position[1];
			u3 = position[2];
			v1 = position[3];
			v2 = position[4];
			v3 = position[5];
			period = position[6];
			altitude = position[7];
			phase = position[8];

			//calculate cartesian coordinates
			cartesianCoordinates = new double[3];
			cartesianCoordinates[0] = (R + altitude)
					* (u1 * Math.cos(2 * pi * time / period + phase)
							+ v1 * Math.sin(2 * pi * time / period + phase));
			cartesianCoordinates[1] = (R + altitude)
					* (u2 * Math.cos(2 * pi * time / period + phase)
							+ v2 * Math.sin(2 * pi * time / period + phase));
			cartesianCoordinates[2] = (R + altitude)
					* (u3 * Math.cos(2 * pi * time / period + phase)
							+ v3 * Math.sin(2 * pi * time / period + phase));
		}

	}
	/**
	 * 24 Satellite positions are initialized based on the time given. Vehicle postion is needed to calculate signal times.
	 * @param vehiclePosition
	 * @param time
	 */
	public Satellites(double[] vehiclePosition, double time) {

		//initializ class variables
		this.time = time;
		this.vehiclePosition = vehiclePosition;
		satList = new Satellite[24];

		//read from data file
		Scanner fileIn;
		try {
			fileIn = new Scanner(new File("data.dat"));

			//initialize more class variables
			pi = Double.parseDouble(fileIn.nextLine().substring(0, 26));
			c = Double.parseDouble(fileIn.nextLine().substring(0, 26));
			R = Double.parseDouble(fileIn.nextLine().substring(0, 26));
			//skip sidereal day data
			fileIn.nextLine(); 
			
			// next line after this will be satellite 0
			//initialize all 24 satelites and store them in satList
			for (int i = 0; i < 24; i++) {
				double[] satPosition = new double[9];
				satPosition[0] = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[1] = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[2] = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[3] = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[4] = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[5] = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[6] = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[7]  = Double.parseDouble(fileIn.nextLine().substring(0, 26));
				satPosition[8] = Double.parseDouble(fileIn.nextLine().substring(0, 26));

				Satellite sat = new Satellite(satPosition);
				satList[i] = sat;
			}

			fileIn.close();
		} 
		catch (FileNotFoundException e) {
			System.out.println("Error with data file");
			System.exit(0);
		}
		catch (NoSuchElementException e) {
			System.out.println("Error with data file");
			System.exit(0);
		}
		catch (NumberFormatException e) {
			System.out.println("Error with data file");
			System.exit(0);
		}
	}

	public ArrayList<Double[]> sendSignal(){
		
		ArrayList<Double[]> signals = new ArrayList<Double[]>();
		for(Satellite sat : satList){
			if(HorizonCheck(sat)){
				Double[] signal = new Double[4];
				signal[0] = findSignalTime(sat);
				signal[1] = sat.cartesianCoordinates[0];
				signal[2] = sat.cartesianCoordinates[1];
				signal[3] = sat.cartesianCoordinates[2];

				signals.add(signal);
			}
		}
		return signals;
	}

	// Returns true if above horizon
	private boolean HorizonCheck(Satellite sat) {
		double xTs = 0.0;
		double xTx = 0.0;
		for (int i = 0; i < 3; i++) {
			xTs = xTs + (vehiclePosition[i] * sat.cartesianCoordinates[i]);
			xTx = xTx + (vehiclePosition[i] * vehiclePosition[i]);
		}
		return xTs > xTx;
	}

	private double findSignalTime(Satellite sat) {
		
		double dx = sat.cartesianCoordinates[0] - vehiclePosition[0];
		double dy = sat.cartesianCoordinates[1] - vehiclePosition[1];
		double dz = sat.cartesianCoordinates[2] - vehiclePosition[2];

		double distance = Math.sqrt((dx * dx) + (dy * dy) + (dz * dz));
		double sigTime = time - (distance/c);
		return sigTime;
	}

}
