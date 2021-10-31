#include <iostream>
#include <string>
#include "Aria.h"

using namespace std;

ArRobot robot;
ArSonarDevice sonar;

int acc = 50;
int dcc = 20;
int rotacc = 10;
int rotdcc = 5;
int dangerrange = 600;
int AbsoluteMaxTransVel = 300;
int AbsoluteMaxTransNegVel = -300;
int AbsoluteMaxRotVel = 90;
int TransVelMax = 50;
int TransNegVelMax = -50;
int RotVelMax = 10;

void Forward()
{
	float SetVel = 0;
	float CurrentVel = robot.getVel();
	SetVel = CurrentVel + acc;
	if (SetVel > robot.getTransVelMax())
	{	
		SetVel = robot.getTransVelMax(); 
	}
	robot.setVel(SetVel);
}

void LeftPointTurn()
{
	float SetRot = 0;
	float CurrentRot = robot.getRotVel();
	SetRot = CurrentRot + rotacc;
	//cout << robot.getRotVel() << robot.getRotVelMax() << endl;
	if (SetRot > robot.getRotVelMax())
	{
		SetRot = robot.getRotVelMax(); 
	}
	robot.setRotVel(SetRot);

}

void RightPointTurn()
{
	float SetRot = 0;
	float CurrentRot = robot.getRotVel();
	SetRot = CurrentRot - rotacc;
	//cout << robot.getRotVel() << robot.getRotVelMax() << endl;
	if (SetRot < -robot.getRotVelMax())
	{
		SetRot = -robot.getRotVelMax(); 
	}
	robot.setRotVel(SetRot);

}

void Backward()
{
	float SetVel = 0;
	float CurrentVel = robot.getVel();
	SetVel = CurrentVel - acc;
	if (SetVel < robot.getTransNegVelMax())
	{
		SetVel = robot.getTransNegVelMax(); 
	}
	robot.setVel(SetVel);
}

void Stop(void)
{
	robot.stop();
}

void DecreaseVel()
{	
	float SetVel = 0;
	float CurrentVel = robot.getVel();
	if (CurrentVel > 0)
	{
		SetVel = CurrentVel - dcc;
		if (SetVel < 0) SetVel = 0;
	}
	else
	{
		SetVel = CurrentVel + dcc;
		if (SetVel > 0) SetVel = 0;
	}
	robot.setVel(SetVel);

}

void DecreaseRotVel()
{	
	float SetRot = 0;
	float CurrentRot = 0;
	CurrentRot = robot.getRotVel();
	if (CurrentRot > 0)
	{
		SetRot = CurrentRot - rotdcc;
		if (SetRot < 0) SetRot = 0;
	}
	else
	{
		SetRot = CurrentRot + rotdcc;
		if (SetRot > 0) SetRot = 0;
	}
	robot.setRotVel(SetRot);

}

double** GetSonarReading()
{
	double** readings = 0;
	readings = new double*[16];

	for (int i=0; i < 16; i++)
	{
		readings[i] = new double[2];
		ArSensorReading s = *robot.getSonarReading(i);
		double r = s.getRange();
		double th =  s.getSensorTh();
		readings[i][0] = r;
		readings[i][1] = th;
	}
	return readings;
}

void BumpingAvoid(double** rs)
{
	bool setflag = 0;
	robot.setTransVelMax(AbsoluteMaxTransVel);
	robot.setTransNegVelMax(AbsoluteMaxTransNegVel);
	robot.setRotVelMax(AbsoluteMaxRotVel);

	for (int i=0; i < 16; i++)
	{
		if (rs[i][0] < dangerrange)
		{
			setflag = 1;

		}
	}

	if (setflag)
	{
		robot.setTransVelMax(TransVelMax);
		robot.setTransNegVelMax(TransNegVelMax);
		robot.setRotVelMax(RotVelMax);	
	}
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
  	if(!robotConnector.connectRobot())
  	{
    	ArLog::log(ArLog::Terse, "part_d: Could not connect to the robot.");
    	if(parser.checkHelpAndWarnUnparsed())
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

    ArLog::log(ArLog::Normal, "part_d: Connected to robot.");

	robot.comInt(ArCommands::ENABLE, 1);

  	// Start the robot processing cycle running in the background.
  	// True parameter means that if the connection is lost, then the 
	// run loop ends.
	robot.runAsync(false);
	robot.setAbsoluteMaxTransVel(AbsoluteMaxTransVel);
	robot.setAbsoluteMaxTransNegVel(AbsoluteMaxTransNegVel);
	robot.setAbsoluteMaxRotVel(AbsoluteMaxRotVel);	


  	// Connect to sonar(s) as defined in parameter files.
 	if(!sonarConnector.connectSonars())
  	{
    	ArLog::log(ArLog::Terse, "Could not connect to configured sonars. Exiting.");
    	Aria::exit(3);
    	return 3;
  	}

  	// Allow some time to read sonar data
  	ArUtil::sleep(500);
 	ArLog::log(ArLog::Normal, "Connected to all sonars.");

	ArKeyHandler keyHandler;
	ArGlobalFunctor ForwardCB(&Forward);
	ArGlobalFunctor LeftPointTurnCB(&LeftPointTurn);
	ArGlobalFunctor RightPointTurnCB(&RightPointTurn);
	ArGlobalFunctor BackwardCB(&Backward);
	ArGlobalFunctor StopCB(&Stop);

	keyHandler.addKeyHandler('i', &ForwardCB);
	keyHandler.addKeyHandler('j', &LeftPointTurnCB);
	keyHandler.addKeyHandler('l', &RightPointTurnCB);
	keyHandler.addKeyHandler(',', &BackwardCB);
	keyHandler.addKeyHandler('k', &StopCB);


	Aria::setKeyHandler(&keyHandler);
	// ArRobot contains an exit action for the Escape key. It also 
	// stores a pointer to the keyhandler so that other parts of the program can
	// use the same keyhandler.
	robot.attachKeyHandler(&keyHandler);
	printf("You may press escape to exit\n");

	while(true)
	{
		printf("X: %.0f Y: %.0f Th: %.1f Vel: %.0f RotVel: %.0f\n", robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel());
		ArUtil::sleep(300);
		if (robot.getVel() !=0)
		{
			DecreaseVel();
		}
		if (robot.getRotVel() !=0 )
		{
			DecreaseRotVel();
		}
		BumpingAvoid(GetSonarReading());

	}

	// End of controling
	Aria::shutdown();
	Aria::exit(0);
}

