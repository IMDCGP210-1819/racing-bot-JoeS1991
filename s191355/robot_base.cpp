/***************************************************************************

    file                 : robot_base.cpp
    created              : Mon 13 Feb 11:40:23 GMT 2017
    copyright            : (C) 2002 Author

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <vector>
#include <iostream>
#include <list>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt);



/* 
 * Module entry point  
 */ 
extern "C" int 

s191355(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("s191355");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 


static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 


static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 


static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
	
} 



//I've opted to try and build this AI using a Finite State Machine.
enum driverStates
{
	Accelerating,
	Slowing,
	Cornering,
};

driverStates currentState = Accelerating;

int timeInCurrentGear = 0;

int distToCorner = 0;

int maxAllowedCornerSpeed = 20;

static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 

	int currentGear = car->_gear;
	int currentSpeed = car->_speed_x;

	
	//If the current track segment is a corner
	if (car->_trkPos.seg->type == TR_LFT || car->_trkPos.seg->type == TR_RGT)
	{
		currentState = Cornering;
	}

	//If the current track segment is a straight
	else if (car->_trkPos.seg->type == TR_STR)
	{
		currentState = Accelerating;
	}

	//Track types are stored as ints, and defined as 
	//		TR_RGT = 1 
	//		TR_LFT = 2 
	//		TR_RGT = 3
	//within track.h, making this for loop and list iteration nice and simple.

	std::list<trackSeg *> nextThirtySegs;
	trackSeg * currSeg = car->_trkPos.seg;

	for (int i = 0; i < 20; i++)		
	{
		nextThirtySegs.push_front(currSeg);
		currSeg = currSeg->next;
	}

	//The below is a range-based for loop, apparently introduced with C++11, more readable for me 
	//than using something like 'for (nextTenSegs::iterator i = nextTenSegs.begin(); i != nextTenSegs.end(); i++){}'

	//We check the next 30 segments for a corner, calculating the length until we find
	//a corner, and when we find one we break the loop 
	for (auto const& i : nextThirtySegs)
	{
		if (i->type == TR_RGT || i->type == TR_LFT)
		{
			currentState = Slowing;
			//std::cout << "corner detected" << std::endl;
			//break;
		}
		else
		{
			distToCorner += i->length;
			maxAllowedCornerSpeed = distToCorner / 3;
		}
	}

	//We clear the list for the next drive() call.
	nextThirtySegs.clear();

	//Tells us the angle of the car.
	float carAngle = car->_yaw;
	
	//Tells us (in metres) how far we are from the middle 'line' of the current track segment,
	//we then divide it by the width of the track segment to avoid oversteering.
	float disToMiddle = car->_trkPos.toMiddle / car->_trkPos.seg->width;

	//Tells us the width of the current track segment (in metres).
	float segWidth = car->_trkPos.seg->width;

	//This calculates the turning angle of the current track segment in radians.
	float angleOfTrack = RtTrackSideTgAngleL(&(car->_trkPos));
	
	float steering = angleOfTrack - carAngle - disToMiddle;
	
	//Bringing the steering value to within -PI / PI, found within tgf.h
	NORM_PI_PI(steering);

	//the 'car->_steerLock' ensures the value of the steering stays within -1 to +1.
	car->_steerCmd = steering / car->_steerLock;


	if (currentState = Accelerating)
	{

		//std::cout << "Accelerating" << std::endl;
		car->ctrl.brakeCmd = 0.0;
		car->_accelCmd = 0.4;

		if (car->_enginerpm >= 850.0)
		{
			currentGear++;
			timeInCurrentGear = 0;
		}
		//The 'timeInCurrentGear' avoided a situation where immediately after changing up
		//a gear, we'd be within the threshhold to change down a gear and get stuck
		//changing up / down between two gears on every tick the whole time.
		else if (car->_enginerpm <= 450.0 && currentGear >= 2 && timeInCurrentGear >= 50)
		{
			currentGear--;
			timeInCurrentGear = 0;
		}
	}
	else if (currentState = Slowing)
	{
		//Stops us from calculating '0' as a max speed and not moving at all.
		if (maxAllowedCornerSpeed < 20)
		{
			maxAllowedCornerSpeed = 20;
		}
		
		//std::cout << "Slowing" << std::endl;
		if (currentSpeed > maxAllowedCornerSpeed)
		{
			car->ctrl.brakeCmd = 0.4;
		}
		else
		{
			car->ctrl.brakeCmd = 0.0;
			car->_accelCmd = 0.4;

			if (car->_enginerpm >= 850.0)
			{
				currentGear++;
				timeInCurrentGear = 0;
			}
			else if (car->_enginerpm <= 450.0 && currentGear >= 2 && timeInCurrentGear >= 50)
			{
				currentGear--;
				timeInCurrentGear = 0;
			}
		}
	}
	else if (currentState = Cornering)
	{
		//std::cout << "Cornering" << std::endl;

		car->_brakeCmd = car->_trkPos.seg->next->arc * 2;
		car->_accelCmd = 0.4;

		if (car->_enginerpm >= 600 && currentGear >= 2)
		{
			currentGear--;
			timeInCurrentGear = 0;
		}
	}

	timeInCurrentGear++;

	distToCorner = 0;

	car->_gearCmd = currentGear;

	//std::cout << car->_enginerpm << std::endl;
	//std::cout << timeInCurrentGear << std::endl;
	//std::cout << car->_trkPos.seg->type << std::endl;
	//std::cout << currentState << std::endl;
	//std::cout << maxAllowedCornerSpeed << std::endl;
	//std::cout << currentSpeed << std::endl;
}

static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

static void
shutdown(int index)
{
}

