/*
 * sensorTile.c
 *
 *  Created on: Mar 24, 2014
 *      Author: jamesm
 */

#include <string.h>
#include <stdio.h>
#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"


const TileGeometryInfo tileGeometryData[] = {
/*  geometry type,					distance to center */
	{TILE_GEOMETRY_SQUARE_LARGE,	TILE_WIDTH_SQUARE_LARGE, 	MILLIRAD_DEG_90},
	{TILE_GEOMETRY_SQUARE_SMALL,	TILE_WIDTH_SQUARE_SMALL, 	MILLIRAD_DEG_90},
	{TILE_GEOMETRY_HEX_LARGE, 		TILE_WIDTH_HEX_LARGE, 		MILLIRAD_DEG_60},
	{TILE_GEOMETRY_HEX_SMALL, 		TILE_WIDTH_HEX_SMALL, 		MILLIRAD_DEG_60},
	{TILE_GEOMETRY_NULL, 			0, 			0}
};


const TileInfo tileData[] = {
/*  ID,		type, 		name, 	geometry type,	default rotation */
	{27, 	TILE_1,		"1", 	TILE_GEOMETRY_HEX_LARGE, 	-MILLIRAD_DEG_30 * 2},
	{255, 	TILE_2,		"2", 	TILE_GEOMETRY_HEX_LARGE, 	-MILLIRAD_DEG_30 * 2},
	{172, 	TILE_3,		"3", 	TILE_GEOMETRY_HEX_LARGE, 	-MILLIRAD_DEG_30 * 2},
	{74, 	TILE_4,		"4", 	TILE_GEOMETRY_HEX_LARGE, 	-MILLIRAD_DEG_30 * 2},
	{51, 	TILE_PLUS,	"+", 	TILE_GEOMETRY_HEX_LARGE, 	0},
	{135, 	TILE_EQUAL,	"=", 	TILE_GEOMETRY_HEX_LARGE,	0},
	{142,   TILE_MULT,  "x",	TILE_GEOMETRY_HEX_LARGE, 	0},
	{19,    TILE_DIV,   "÷",	TILE_GEOMETRY_HEX_LARGE, 	0},
	{0, 	TILE_NULL,	"?", 	0,				 			0}
};


TileInfo* tileReadSensor(void) {
	uint8 tileID = bumpSensorsGetBits();
	uint8 tileIdx = 0;
	uint8 tileIDList;
	const TileInfo* tilePtr = NULL;
//	cprintf("tileID %d\n", tileID);
	do {
		tileIDList = tileData[tileIdx].tileID;
		if (tileIDList == TILE_NULL) {
			break;
		}
		if (tileIDList == tileID) {
			tilePtr = &tileData[tileIdx];
			break;
		}
		tileIdx++;
	} while (TRUE);
	return (TileInfo*)tilePtr;
}


char* tilePrint(TileInfo* tilePtr) {
	static char printString[20];
	if (tilePtr) {
		sprintf(printString, "%s", tilePtr->name);
	} else {
		sprintf(printString, "null");
	}
	return printString;
}


TileGeometryInfo* tileGetGeometryInfo(TileInfo* tilePtr) {
	uint8 i = 0;
	TileGeometryInfo* geometryPtr = NULL;
	if(tilePtr) {
		geometryPtr = (TileGeometryInfo*)&tileGeometryData[i];
		do {
			if (tilePtr->tileGeometry == geometryPtr->tileGeometry) {
				break;
			}
			geometryPtr = (TileGeometryInfo*)&tileGeometryData[i];
			i++;
		} while (geometryPtr->tileGeometry != TILE_GEOMETRY_NULL);
		return geometryPtr;
	} else {
		return NULL;
	}
}

uint8 tileGetID(TileInfo* tilePtr) {
	if(tilePtr) {
		return tilePtr->tileID;
	} else {
		return 0;
	}
}

uint8 tileGetType(TileInfo* tilePtr) {
	if(tilePtr) {
		return tilePtr->tileType;
	} else {
		return TILE_NULL;
	}
}

const char* tileGetName(TileInfo* tilePtr) {
	if(tilePtr) {
		return tilePtr->name;
	} else {
		return "";
	}
}

int16 tileGetDistanceToCenter(TileInfo* tilePtr) {
	if(tilePtr) {
		TileGeometryInfo* geometryPtr = tileGetGeometryInfo(tilePtr);
		if (geometryPtr) {
			return geometryPtr->distanceToCenter;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}

int16 tileGetRotation(TileInfo* tilePtr) {
	if(tilePtr) {
		return tilePtr->defaultRotation;
	} else {
		return 0;
	}
}

