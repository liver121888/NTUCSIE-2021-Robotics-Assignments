#include <iostream>
#include <string>
#include "Aria.h"

using namespace std;

ArRobot robot;
ArSonarDevice sonar;

bool Navigator(ArPose *TargetPose)
{

	double angle = robot.findAngleTo(*TargetPose);
	double distance = robot.findDistanceTo(*TargetPose);

	robot.setHeading(angle);
	while (!robot.isHeadingDone())
	{
	}
	robot.move(distance);
	while (!robot.isMoveDone())
	{
	}
	robot.setHeading(TargetPose->getTh());
	while (!robot.isHeadingDone())
	{
	}
	return robot.isHeadingDone();
}

int main(int argc, char **argv)
{

	robot.addRangeDevice(&sonar);
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	ArSonarConnector sonarConnector(&parser, &robot, &robotConnector);

	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.
	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "part_e: Could not connect to the robot.");
		if (parser.checkHelpAndWarnUnparsed())
		{
			// -help not given
			Aria::logOptions();
			Aria::exit(1);
		}
	}
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(2);
		return 2;
	}

	ArLog::log(ArLog::Normal, "part_e: Connected to robot.");

	robot.comInt(ArCommands::ENABLE, 1);

	// Start the robot processing cycle running in the background.
	// True parameter means that if the connection is lost, then the
	// run loop ends.
	robot.runAsync(false);

	// Connect to sonar(s) as defined in parameter files.
	if (!sonarConnector.connectSonars())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured sonars. Exiting.");
		Aria::exit(3);
		return 3;
	}

	// Allow some time to read sonar data
	ArUtil::sleep(500);
	ArLog::log(ArLog::Normal, "Connected to all sonars.");

	ArPose *initpose = new ArPose(5090, 3580, 3093.97);
	robot.moveTo(*initpose);
	ArPose* pose;
	char ans;
	do
	{
		cout << "Please input the destination in as the format(x/m y/m theta/rads):  ";
		double x, y, theta;
		cin >> x;
		cin >> y;
		cin >> theta;
		x = x * 1000.0;
		y = y * 1000.0; 
		theta =  theta * 180.0/M_PI;
		pose = new ArPose(x, y,theta);
		Navigator(pose);
		printf("X: %f Y: %f Th: %f\n", robot.getX(), robot.getY(), robot.getTh());
		ArUtil::sleep(300);
		cout << "Do you wish to set next target? [y/n] ";

		cin >> ans;

	} while (ans == 'y');
	cout << "Moving back to initial position\n";
	Navigator(initpose);
	cout << "End of controling...\n";
	// End of controling
	Aria::shutdown();
	Aria::exit(0);
}