# GPS Simulator
GPS Simulator is a package of Java programs that simulate the calculations performed by a real GPS system to approximate a user's location. The package includes the following programs and files:

* Main: Handles the flow of data between Satellites and Receiver programs and serves as the entry point for the package.
* Satellites: Provides information about satellite positions and calculates necessary data for signal transmission.
* Receiver: Analyzes satellite information to approximate the user's location.
* Vehicle-Position.txt: Contains the actual location of the vehicle and receiver at a given time.
* Approximate-Position.txt: Displays the approximated location calculated by the Receiver program.

# Simulation Overview
In a real GPS system, satellites orbiting the Earth send information to a receiver about their positions and the time at which they transmitted the signals. The receiver uses the time difference between signal transmission and reception to calculate its distance from each satellite and determine its own location. This simulation can't rely on the actual time difference between signals being sent and received. Instead, the Satellites program is given information about the vehicle's position at a specific time. Using this information, it reconstructs the data required by a receiver. True to a real GPS, the Receiver program does not know the vehicle's location. It approximates the vehicle's location using only the information sent by the Satellites program.

# Coordinate Systems
Two coordinate systems are used in these programs. The standard geographical coordinate system employs altitude, degrees of longitude, and degrees of latitude to describe positions on the Earth's surface. The other coordinate system is a 3-dimensional Cartesian system, representing the larger space where satellites orbit the Earth and the Earth itself rotates. The origin of this coordinate system is the center of the Earth, with the Earth rotating around the Z-axis. Assuming the North Pole is at sea level, its Cartesian coordinates would be (0,0,R), where R is the radius of the Earth. At the start of each sidereal day, the point with 0 longitude, latitude, and altitude is represented by the Cartesian coordinates (R,0,0). Positions for input and output in the programs are given in geographical coordinates but are converted to Cartesian coordinates for intermediate calculations. Satellite locations are always provided in Cartesian coordinates.

# Program Descriptions
## Main
Main acts as the central program that coordinates the data flow between Satellites and Receiver. It reads the actual vehicle location from the Vehicle-Position.txt file and prints the approximated location to the Approximate-Position.txt file. To run the program, execute the Main file and view the approximation in the output text file.

## Satellites
Satellites program stores location information for 24 satellites. Given a time and the vehicle's position, this program determines which of the 24 satellites are above the horizon from the vehicle's perspective at that time. It then calculates the necessary information for each satellite's signal, including the satellite's position in Cartesian coordinates and the time at which the signal should be transmitted for it to reach the vehicle at the specified time. This information is passed to the Main program and subsequently to the Receiver program.

## Receiver
Receiver analyzes the information provided by the Satellites program to approximate the original position of the vehicle. Without access to the actual position given in Vehicle-Position.txt, Receiver utilizes positional data of each satellite to construct a system of equations with four unknowns: the vehicle's three Cartesian coordinates and the time the vehicle is at that position. To approximate a solution, Receiver applies Newton's method for numerical approximation, an iterative method known for its efficiency. However, to start the iteration, the Receiver requires a known location close to the vehicle. In practical terms, a real GPS system would use its last known location as a starting point. Once the iteration is complete, Receiver sends the approximated position back to the Main program for output.

# Usage
1. Compile and run the Main program.
2. View the approximated vehicle location in the Approximate-Position.txt file.

# Contact Information
If you have any questions or feedback regarding the GPS Simulator, please feel free to contact the developer at saleemasqazi@gmail.com
