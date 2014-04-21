/*
 * sensorTile.h
 *
 *  Created on: Mar 24, 2014
 *      Author: jamesm
 */

#ifndef SENSORTILE_H_
#define SENSORTILE_H_

//#define TILEID_NULL		0
//#define TILEID_1		27
//#define TILEID_2		255
//#define TILEID_3		172
//#define TILEID_4		74
//#define TILEID_PLUS		51
//#define TILEID_EQUAL	135

#define TILE_NULL		0
#define TILE_1			1
#define TILE_2			2
#define TILE_3			3
#define TILE_4			4
#define TILE_PLUS		100
#define TILE_MINUS		101
#define TILE_MULT		102
#define TILE_DIV		103
#define TILE_EQUAL		110

#define TILE_GEOMETRY_NULL				0
#define TILE_GEOMETRY_SQUARE_LARGE		1
#define TILE_GEOMETRY_SQUARE_SMALL		2
#define TILE_GEOMETRY_HEX_LARGE			3
#define TILE_GEOMETRY_HEX_SMALL			4

#define TILE_WIDTH_SQUARE_LARGE			(112/2)
#define TILE_WIDTH_SQUARE_SMALL			(106/2)
#define TILE_WIDTH_HEX_LARGE			(192/2)
#define TILE_WIDTH_HEX_SMALL			(182/2)


typedef struct TileGeometryInfo {
	uint8 tileGeometry;
	int16 distanceToCenter;
	int16 degreesOfRotation;
} TileGeometryInfo;

typedef struct TileInfo {
	uint8 tileID;
	uint8 tileType;
	const char* name;
	uint8 tileGeometry;
	int16 defaultRotation;
} TileInfo;


TileInfo* tileReadSensor(void);
char* tilePrint(TileInfo* tilePtr);

uint8 tileGetID(TileInfo* tilePtr);
uint8 tileGetType(TileInfo* tilePtr);
const char* tileGetName(TileInfo* tilePtr);
int16 tileGetDistanceToCenter(TileInfo* tilePtr);
int16 tileGetRotation(TileInfo* tilePtr);
TileGeometryInfo* tileGetGeometryInfo(TileInfo* tilePtr);

#endif /* SENSORTILE_H_ */
