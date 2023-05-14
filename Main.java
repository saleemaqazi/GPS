import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Scanner;

/**
 * This class is a launch pad for a gps simulation. It reads a vehicle's true position and data about the 
 * movement of the earth from a couple files. Then it creates satellite objects that send signals to a 
 * reciever. The reciever approximates the vehicle's position and writes it to a file.
 */
public class Main {
    private static double s,R;
    
	public static void main(String[] args) {
        //read radius of earth and length of sidereal day from data file
		Scanner fileIn;
		try {
			fileIn = new Scanner(new File("data.dat"));
			//skip line with value of pi
			fileIn.nextLine();
            //skip line with value of spped of light
			fileIn.nextLine();
			R = Double.parseDouble(fileIn.nextLine().substring(0, 26));
            s = Double.parseDouble(fileIn.nextLine().substring(0, 26));
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

        //A start location is needed for newton's method, this one is in slc
        double[] startLocationGeographic = new double[]{12123.0, 40, 45, 55, 1, 111, 50, 58, -1, 1372};
        double[] startLocationCartesian = convertToCartesian(startLocationGeographic);

        //read location of vehicle from file
		double[] vehiclePositionGeographic = readVehicleInput();
        double[] vehiclePosition = convertToCartesian(vehiclePositionGeographic);

        //create satellites and send a signal to a reciever
        Satellites satellites = new Satellites(vehiclePosition, vehiclePositionGeographic[0]);
        ArrayList<Double[]> signal = satellites.sendSignal();
        Receiver reciever = new Receiver(signal, startLocationCartesian);
        double[] approxVehiclePosition = reciever.getVehiclePosition();
        double[] approxPositionGeographic = convertToGeographic(approxVehiclePosition, vehiclePositionGeographic[0]);
        
        //write approximate location to a file
        try {
            BufferedWriter output = new BufferedWriter(new FileWriter("Approximate-Position.txt"));
            for(Double d : approxPositionGeographic){
                output.write(d + " ");
            }
            output.close();
        } 
        catch (IOException e) {
            System.out.println("Error with output file");
            e.printStackTrace();
        }
	}

    /**
     * Reads vehicle position file and returns a double array containing the geographical coordinates 
     * of the vehicle in the from {time, latitude degrees, latitude minutes, latitude seconds, 
     * North South (+/- 1), longitude degrees, longitude minutes, longitude seconds, East West (+/- 1), height}
     */
	private static double[] readVehicleInput() {
		double[] input = new double[10];
		Scanner sc;
		try {
			sc = new Scanner(new File("Vehicle-Position.txt"));
			for (int i = 0; i < 10; i++) {
				input[i] = sc.nextDouble();
			}
			sc.close();

		} catch (FileNotFoundException e) {
			System.out.println("Vehicle Position file not found");
			System.exit(0);
		}

		return input;
	}

    /**
     * Takes cartesian coordinates {x,y,z} and converts them to geographical coordinates
     * {time, latitude degrees, latitude minutes, latitude seconds, North South (+/- 1), 
     * longitude degrees, longitude minutes, longitude seconds, East West (+/- 1), height}
     */
    private static double[] convertToGeographic(double[] cartesianCoordinates, double time){
		 
        double x = cartesianCoordinates[0];
        double y = cartesianCoordinates[1];
        double z = cartesianCoordinates[2];
        double t = time;
        double a = - (2 * Math.PI * t) / s;
        double xv = (Math.cos(a) * x) - (Math.sin(a) * y);
        double yv = (Math.sin(a) * x) + (Math.cos(a) * y);
        double zv = z;
        double h = Math.sqrt((xv * xv) + (yv * yv) + (zv * zv)) - R;
        double lat;
        if(x * x + y * y != 0)
            lat = Math.atan(zv / Math.sqrt(xv * xv + yv * yv));
        else if(zv > 0)
            lat = Math.PI / 2.0;
        else
            lat = - Math.PI / 2.0;

        double NS = Math.signum(zv);
        double EW = Math.signum(yv);

        double temp = Math.abs((lat * 180) / Math.PI);
        double latD = (double)(int)temp;
        temp = ((temp - latD) * 60);
        double latM = (double)(int)temp;
        double latS = ((temp - latM) * 60);

        temp = Math.abs((Math.acos(xv / (Math.cos(lat) * (R + h))) * 180.0) / Math.PI);
        double lonD = (double)(int) temp;
        temp = ((temp - lonD) * 60);
        double lonM = (double)(int)temp;
        double lonS = ((temp - lonM) * 60);

        return new double[]{t, latD, latM, latS, NS, lonD, lonM, lonS, EW, h};
    }

    /**
     * Takes geographical coordinates {time, latitude degrees, latitude minutes, 
     * latitude seconds, North South (+/- 1), longitude degrees, longitude minutes, 
     * longitude seconds, East West (+/- 1), height} and converts them to cartesian coordinates {x,y,z}
     */
    private static double[] convertToCartesian(double [] goegraphicalCoordinates) {
        //rename for formula readability
        double t, latD, latM, latS, NS, lonD,
            lonM, lonS, EW, h;
        t = goegraphicalCoordinates[0];
        latD = goegraphicalCoordinates[1];
        latM = goegraphicalCoordinates[2];
        latS = goegraphicalCoordinates[3];
        NS = goegraphicalCoordinates[4];
        lonD = goegraphicalCoordinates[5];
        lonM = goegraphicalCoordinates[6];
        lonS = goegraphicalCoordinates[7];
        EW = goegraphicalCoordinates[8];
        h = goegraphicalCoordinates[9];

        // add degrees minutes and seconds for longitude and latitude
        double lat = 2 * Math.PI * NS * ((latD / 360.0) + (latM / (360.0 * 60.0)) + (latS / (360.0 * 60.0 * 60.0)));
        double lon = 2 * Math.PI * EW * ((lonD / 360.0) + (lonM / (360.0 * 60.0)) + (lonS / (360.0 * 60.0 * 60.0)));

        // find x, y, z prior to rotation
        double x = (6367444.50 + h) * Math.cos(lat) * Math.cos(lon);
        double y = (6367444.50 + h) * Math.cos(lat) * Math.sin(lon);
        double z = (6367444.50 + h) * Math.sin(lat);

         
        //create return value
        double[] cartCoord = new double[3];

        // multiply by rotational matrix to account for earths daily rotation
        double a = (2 * Math.PI * t) / 8.616408999999999651E+04;
        cartCoord[0] = (Math.cos(a) * x) - (Math.sin(a) * y);
        cartCoord[1] = (Math.sin(a) * x) + (Math.cos(a) * y);
        cartCoord[2] = z;

        return cartCoord;
    }
}
