import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Scanner;
import java.lang.Math;

/**
 * This class takes a group of signals from several satellites and uses the infromation 
 * to approximate the position of a vehicle. The satellites positions are used to set up
 * a system of nonlinear equations wich is then solved using Newton's method for numerical
 * approximation. Convergence of Newton's method requires an initial position that is relatively
 * close to the true solution. If the vehicle is moving then the last known location can be
 * used as the initial position.  
 */
class Receiver{
	private static double c;
	private ArrayList<Double[]> sats;
	private double[] vehiclePostion;

	public Receiver(ArrayList<Double[]> satelliteSignals, double[] lastKnownLocation){

		sats = satelliteSignals;
		
		//read spped of light from data file
		Scanner fileIn;
		try {
			fileIn = new Scanner(new File("data.dat"));
			//skip line with value of pi
			fileIn.nextLine();
			c = Double.parseDouble(fileIn.nextLine().substring(0, 26));
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

		vehiclePostion = newton(lastKnownLocation);
	}

	/**
	 * Returns an approximation of the vehicle position based on the satellite signals sent to this reciever.
	 * The position is given in cartesian coordinates.
	 * @return vehicle position in form {x,y,z}
	 */
	public double[] getVehiclePosition(){
		return vehiclePostion;
	}
	
	/**
	 * Performs newton's method to approximate the solution a system of nonlinear 
	 * equations based on the locations of the satellites that initialized this reciever.
	 * The start position must be relatively close to the actual solution to ensure convergence.
	 * @param start a location given in cartesian coordinates that is relatively close to the actual solution.
	 * @return approximate vehicle position in cartesian coordinates {x,y,z}
	 */
	private double[] newton(double[] start) {
		double[] x;
		double[] xn = start;
		double[][] jacobian;
		double[] grad;
		double[] s;
		
		do {
			x = xn;
			jacobian = J(x[0], x[1], x[2]);
			grad = F(x[0], x[1], x[2]);
			s = gauss(jacobian, neg(grad));
			xn = add(x,s);
			
		} while(normCheck(x,xn) > .01);
		
		return xn;
	}
	
	/*
	 * Negates every term in a vector and returns the result
	 */
	private double[] neg(double[] x) {
		double[] res = new double[x.length];
		for(int i = 0; i < x.length; i++) {
			res[i] = x[i] * -1;
		}
		
		return res;
	}
	
	/*
	 * Adds two vectors and returns the result
	 */
	private double[] add(double[] x, double[] y) {
		double[] z = new double[x.length];
		
		if (x.length != y.length) {
			return z;
		}
		
		for (int i = 0; i < x.length; i++){
			z[i] = x[i] + y[i];
		}
		
		return z;
	}
	

	/**
	 * Calculates the system of equations:
	 * df/dx, df/dy, df/dz evaluated at given x,y,z
	 * where f(x) = sum for all stellites(||xsi+1 - x|| - ||xsi - x|| - c (tsi -
	 * tsi+1))^2
	 **/
	private double[] F(double x, double y, double z) {
		double[] f = { 0, 0, 0 };

		// loop through all satellites
		for (int i = 0; i < sats.size() - 1; i++) {
			// calculate basic ingredients
			double N1 = Math.sqrt((sats.get(i)[1] - x) * (sats.get(i)[1] - x) +
					(sats.get(i)[2] - y) * (sats.get(i)[2] - y) +
					(sats.get(i)[3] - z) * (sats.get(i)[3] - z));
			double N2 = Math.sqrt((sats.get(i + 1)[1] - x) * (sats.get(i + 1)[1] - x) +
					(sats.get(i + 1)[2] - y) * (sats.get(i + 1)[2] - y) +
					(sats.get(i + 1)[2] - y) * (sats.get(i + 1)[2] - y) +
					(sats.get(i + 1)[3] - z) * (sats.get(i + 1)[3] - z));
			double A = N2 - N1 - (c * (sats.get(i)[0] - sats.get(i + 1)[0]));
			double X = ((sats.get(i)[1] - x) / N1) - ((sats.get(i + 1)[1] - x) / N2);
			double Y = ((sats.get(i)[2] - y) / N1) - ((sats.get(i + 1)[2] - y) / N2);
			double Z = ((sats.get(i)[3] - z) / N1) - ((sats.get(i + 1)[3] - z) / N2);

			// sum each equation and store in vector
			f[0] = f[0] + (A * X);
			f[1] = f[1] + (A * Y);
			f[2] = f[2] + (A * Z);

		}

		// double each sum
		f[0] = f[0] * 2;
		f[1] = f[1] * 2;
		f[2] = f[2] * 2;
		return f;
	}
	
	/**
	 * Creates Jacobain at the point x,y,z based on the system of equations in F
	 **/
	private double[][] J(double x, double y, double z) {
		double[][] j = new double[3][3];
		// loop through satellites
		for (int i = 0; i < sats.size() - 1; i++) {
			// calculate basic ingredients
			double N1 = Math.sqrt((sats.get(i)[1] - x) * (sats.get(i)[1] - x) +
					(sats.get(i)[2] - y) * (sats.get(i)[2] - y) +
					(sats.get(i)[3] - z) * (sats.get(i)[3] - z));
			double N2 = Math.sqrt((sats.get(i + 1)[1] - x) * (sats.get(i + 1)[1] - x) +
					(sats.get(i + 1)[2] - y) * (sats.get(i + 1)[2] - y) +
					(sats.get(i + 1)[3] - z) * (sats.get(i + 1)[3] - z));
			double A = N2 - N1 - (c * (sats.get(i)[0] - sats.get(i + 1)[0]));
			double X = ((sats.get(i)[1] - x) / N1) - ((sats.get(i + 1)[1] - x) / N2);
			double Y = ((sats.get(i)[2] - y) / N1) - ((sats.get(i + 1)[2] - y) / N2);
			double Z = ((sats.get(i)[3] - z) / N1) - ((sats.get(i + 1)[3] - z) / N2);

			// calculate partial derivatives
			double dXX = ((N2 * N2) - ((sats.get(i + 1)[1] - x) * (sats.get(i + 1)[1] - x)))
					/ (N2 * N2 * N2)
					- ((N1 * N1) - ((sats.get(i)[1] - x) * (sats.get(i)[1] - x)))
							/ (N1 * N1 * N1);

			double dYY = ((N2 * N2) - ((sats.get(i + 1)[2] - y) * (sats.get(i + 1)[2] - y)))
					/ (N2 * N2 * N2)
					- ((N1 * N1) - ((sats.get(i)[2] - y) * (sats.get(i)[2] - y)))
							/ (N1 * N1 * N1);

			double dZZ = ((N2 * N2) - ((sats.get(i + 1)[3] - z) * (sats.get(i + 1)[3] - z)))
					/ (N2 * N2 * N2)
					- ((N1 * N1) - ((sats.get(i)[3] - z) * (sats.get(i)[3] - z)))
							/ (N1 * N1 * N1);

			double dXY = ((sats.get(i)[2] - y) * (sats.get(i)[1] - x)
					/ (N1 * N1 * N1))
					- ((sats.get(i + 1)[2] - y) * (sats.get(i + 1)[1] - x)
							/ (N2 * N2 * N2));

			double dXZ = ((sats.get(i)[3] - z) * (sats.get(i)[1] - x)
					/ (N1 * N1 * N1))
					- ((sats.get(i + 1)[3] - z) * (sats.get(i + 1)[1] - x)
							/ (N2 * N2 * N2));

			double dYZ = ((sats.get(i)[2] - y) * (sats.get(i)[3] - z)
					/ (N1 * N1 * N1))
					- ((sats.get(i + 1)[2] - y) * (sats.get(i + 1)[3] - z)
							/ (N2 * N2 * N2));

			// calculate 2nd order derivatives and sum for all satellites
			// populating only half of jacobian since it is symmetric
			j[0][0] = j[0][0] + (X * X) + (A * dXX);
			j[1][0] = j[1][0] + (X * Y) + (A * dXY);
			j[2][0] = j[2][0] + (X * Z) + (A * dXZ);
			j[1][1] = j[1][1] + (Y * Y) + (A * dYY);
			j[2][1] = j[2][1] + (Y * Z) + (A * dYZ);
			j[2][2] = j[2][2] + (Z * Z) + (A * dZZ);
		}

		// Double every sum
		j[0][0] = 2 * j[0][0];
		j[1][0] = 2 * j[1][0];
		j[2][0] = 2 * j[2][0];
		j[1][1] = 2 * j[1][1];
		j[2][1] = 2 * j[2][1];
		j[2][2] = 2 * j[2][2];

		// fill in the other enteries with the symmetric equivalent
		j[0][1] = j[1][0];
		j[0][2] = j[2][0];
		j[1][2] = j[2][1];

		return j;
	}

	/*
	 * Uses Gaussian Elimination with pivoting to solve a 3x3 system of equations
	 */
	private double[] gauss(double[][] A, double[] b) {

		double[] x = new double[3];

		for (int i = 0; i < 3; i++) {

			// pivoting - find max index
			int maxI = i;
			for (int j = i + 1; j < 3; j++) {
				if (Math.abs(A[j][i]) > Math.abs(A[maxI][i])) {
					maxI = j;
				}
			}

			// do switchies (actual pivot part)
			double t = b[i];
			b[i] = b[maxI];
			b[maxI] = t;

			double[] temp = A[i];
			A[i] = A[maxI];
			A[maxI] = temp;

			// check for singularity
			if (Math.abs(A[i][i]) <= 1e-15) {
				throw new ArithmeticException("Singularity during gaussian elimination");
			}

			// reduce under diagonal
			for (int k = i + 1; k < 3; k++) {

				// find what we need to multiply row by to reduce
				double mult = A[k][i] / A[i][i];

				// subtract adjusted row from current row
				b[k] = b[k] - mult * b[i];
				for (int l = i; l < 3; l++) {
					A[k][l] = A[k][l] - mult * A[i][l];
				}
			}
		}

		// backward, work from bottom to solve
		for (int m = 2; m >= 0; m--) {

			double sum = 0.0;

			// at bottom row, we already have solution. work our way back up with sums for
			// the rest
			for (int n = m + 1; n < 3; n++) {
				sum += A[m][n] * x[n];
			}

			// solve the single row equation
			x[m] = (b[m] - sum) / A[m][m];
		}
		return x;
	}

	/*
	 * Finds the 2 norm of the difference between two vectors of the same length. If
	 * different lengths, returns -1
	 */
	private double normCheck(double[] x, double[] y) {
		double res = 0.0;
		if (x.length != y.length) {
			return -1.0;
		}
		for (int i = 0; i < x.length; i++) {
			res += Math.pow((x[i] - y[i]), 2);
		}

		return Math.sqrt(res);
	}
}
