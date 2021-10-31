#include <iostream>
#include <string>
#include "Aria.h"

using namespace std;

ArRobot robot;
ArSonarDevice sonar;
ArActionGroup *avoid;
ArActionGotoStraight *gotoPoseAction;

void Navigator(ArPose *TargetPose)
{

	gotoPoseAction->setGoal(*TargetPose, false, false);
	while (!gotoPoseAction->haveAchievedGoal())
	{
	}
	cout << "Position achieved.\n";
	cout << "Heading.\n";
	double angle = TargetPose->getTh();
	robot.setHeading(angle);
	while (!robot.isHeadingDone())
	{
	}
	cout << "Goal achieved.\n";
	gotoPoseAction->cancelGoal();
	avoid->deactivate();

}

int main(int argc, char **argv)
{
	avoid = new ArActionGroup(&robot);
	gotoPoseAction = new ArActionGotoStraight("gotostraight");
	// if we're stalled we want to back up and recover
	avoid->addAction(new ArActionStallRecover, 100);
	// react to bumpers
	avoid->addAction(new ArActionBumpers, 75);
	// turn to avoid things closer to us
	avoid->addAction(new ArActionAvoidFront("Avoid Front Near", 225, 0), 50);
	// turn avoid things further away
	avoid->addAction(new ArActionAvoidFront, 45);

	avoid->addAction(gotoPoseAction, 10);
	gotoPoseAction->setCloseDist(40);
	avoid->deactivate();

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
		ArLog::log(ArLog::Terse, "bonus: Could not connect to the robot.");
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

	ArLog::log(ArLog::Normal, "bonus: Connected to robot.");

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

	ArPose *initpose = new ArPose(0.0, 0.0, 0.0);
	robot.moveTo(*initpose);
	ArPose *pose;
	cout << "Please input the destination in as the format(x/m y/m theta/rads):  ";
	double x, y, theta;
	cin >> x;
	cin >> y;
	cin >> theta;
	x = x * 1000.0;
	y = y * 1000.0;
	theta = theta * 180.0 / M_PI;
	pose = new ArPose(x, y, theta);
	avoid->activate();
	Navigator(pose);
	printf("X: %f Y: %f Th: %f\n", robot.getX(), robot.getY(), robot.getTh());
	ArUtil::sleep(300);
	cout << "End of controling...\n";
	//End of controling
	Aria::shutdown();
	Aria::exit(0);
}
