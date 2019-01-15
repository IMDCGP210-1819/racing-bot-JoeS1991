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

/* Module interface initialization. */
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

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
	
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
    
	car->ctrl.brakeCmd = 0.0;
	car->_accelCmd = 0.4;

	
	//Creating two vectors, one for the middle position of the next track, and another for the position of the car.
	std::vector<float> nextTrackMiddle = {car->_trkPos.seg->next->center.x, car->_trkPos.seg->next->center.y};
	std::vector<float> carPosition = {car->_pos_X, car->_pos_Y};
	
	//Tells us the angle of the car.
	float carAngle = car->_yaw;
	
	//Tells us (in metres) how far we are from the middle 'line' of the current track segment.
	float disToMiddle = car->_trkPos.toMiddle / car->_trkPos.seg->width;

	//Tells us the width of the current track segment (in metres).
	float segWidth = car->_trkPos.seg->width;

	//This calculates the turning angle of the current track segment in radians.
	float angleOfTrack = RtTrackSideTgAngleL(&(car->_trkPos));
	
	float steering = angleOfTrack - carAngle - disToMiddle;

	vec2d nextWayPoint = car->_trkPos.seg->next->center.x;
	
	NORM_PI_PI(steering);

	car->_steerCmd = steering / car->_steerLock;

	//GearChanger(car);
	int currentGear = car->_gear;
	if (car->_enginerpm >= (car->_enginerpmRedLine * 0.9))
	{
		currentGear++;
	}
	

	if ((car->_enginerpm >= (car->_enginerpmRedLine * 0.5)) && (car->_trkPos.seg->next->arc > 0) && (currentGear >= 3) ||
		(car->_enginerpm >= (car->_enginerpmRedLine * 0.5)) && (car->_trkPos.seg->arc > 0) && (currentGear >= 3))
	{
		car->_brakeCmd = car->_trkPos.seg->next->arc;
		currentGear--;
	}

	car->_gearCmd = currentGear;
	
	/*  
     * add the driving code here to modify the 
     * car->_steerCmd 
     * car->_accelCmd 
     * car->_brakeCmd 
     * car->_gearCmd 
     * car->_clutchCmd 
     */ 
}

static void
GearChanger(tCarElt* car)
{

}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

