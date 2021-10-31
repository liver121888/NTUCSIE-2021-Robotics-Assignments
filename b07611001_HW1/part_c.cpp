#include <iostream>
#include "Aria.h"

using namespace std;

int acc = 50;
int dcc = 20;
int rotacc = 10;
int rotdcc = 5;
int AbsoluteMaxTransVel = 500;
int AbsoluteMaxTransNegVel = -500;
int AbsoluteMaxRotVel = 90;

ArRobot robot;
ArSonarDevice sonar;

void Forward()
{
	float SetVel = 0;
	float CurrentVel = 0;
	CurrentVel = robot.getVel();
	SetVel = CurrentVel + acc;
	robot.setVel(SetVel);
}

//TODO: rotvel upperlimit is weired 100/90
void LeftPointTurn()
{
	float SetRot = 0;
	float CurrentRot =0;
	CurrentRot = robot.getRotVel();
	SetRot = CurrentRot + rotacc;
	if (SetRot > robot.getAbsoluteMaxRotVel())
	{
		SetRot = robot.getAbsoluteMaxRotVel(); 
	}
	robot.setRotVel(SetRot);

}

void RightPointTurn()
{
	float SetRot = 0;
	float CurrentRot = 0;
	CurrentRot = robot.getRotVel();
	SetRot = CurrentRot - rotacc;
	if (SetRot < -robot.getAbsoluteMaxRotVel())
	{
		SetRot = -robot.getAbsoluteMaxRotVel(); 
	}
	robot.setRotVel(SetRot);

}

void Backward()
{
	float SetVel = 0;
	float CurrentVel = 0;
	CurrentVel = robot.getVel();
	SetVel = CurrentVel - acc;
	robot.setVel(SetVel);
}

void Stop(void)
{
	robot.setVel(0);
	robot.setRotVel(0);
}

void DecreaseVel()
{	
	float SetVel = 0;
	float CurrentVel = 0;
	CurrentVel = robot.getVel();
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

int main(int argc, char **argv)
{

	robot.addRangeDevice(&sonar);

	Aria::init();
	ArSimpleConnector connector(&argc,argv);
	if (!connector.connectRobot(&robot))
	{
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		Aria::exit(1);
	}


	robot.comInt(ArCommands::ENABLE, 1);
	robot.runAsync(false);
	robot.setAbsoluteMaxTransVel(AbsoluteMaxTransVel);
	robot.setAbsoluteMaxTransNegVel(AbsoluteMaxTransNegVel);
	robot.setAbsoluteMaxRotVel(AbsoluteMaxRotVel);

	// Used to perform actions when keyboard keys are pressed
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
		printf("Trans. Vel: %f | Rot. Vel: %f \n", robot.getVel(), robot.getRotVel());
		ArUtil::sleep(300);
		if (robot.getVel() !=0)
		{
			DecreaseVel();
		}
		if (robot.getRotVel() !=0 )
		{
			DecreaseRotVel();
		}
	}
	// End of controling
	Aria::shutdown();
	Aria::exit(0);
}