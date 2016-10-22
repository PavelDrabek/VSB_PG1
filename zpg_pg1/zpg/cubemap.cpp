#include "stdafx.h"

CubeMap::CubeMap(std::string directory) {

	std::string filenames[6] = {
		directory + "/posx.jpg",
		directory + "/posy.jpg",
		directory + "/posz.jpg",
		directory + "/negx.jpg",
		directory + "/negy.jpg",
		directory + "/negz.jpg" };

	printf("\nLoading cubemap:\n");
	for (int i = 0; i < 6; i++) {
		printf("%s\n", filenames[i].c_str());
		maps_[i] = LoadTexture((filenames[i]).c_str());
		showed[i] = false;
	}
}

CubeMap::~CubeMap() {
	for (int i = 0; i < 6; i++)
	{
		delete maps_[i];
		maps_[i] = NULL;
	}
}

void CubeMap::PrintShowedTextures() {
	printf("\nShowed textures: \n");
	printf("POS_X = %d \n", showed[POS_X]);
	printf("POS_Y = %d \n", showed[POS_Y]);
	printf("POS_Z = %d \n", showed[POS_Z]);
	printf("NEG_X = %d \n", showed[NEG_X]);
	printf("NEG_Y = %d \n", showed[NEG_Y]);
	printf("NEG_Z = %d \n", showed[NEG_Z]);
}

Color4 CubeMap::get_texel(Vector3 & direction)
{
	int index = direction.LargestComponent(true);
	int map = index + (direction.data[index] < 0 ? 3 : 0);
	showed[map] = true;

	float u, v;
	const float tmp = 1.0f / abs(direction.data[index]);
	switch (map)
	{
	case POS_X: 
		u = 1-(direction.z * tmp + 1) * 0.5f;
		v = (direction.y * tmp + 1) * 0.5f;
		break;
	case POS_Y: 
		u = (direction.x * tmp + 1) * 0.5f;
		v = 1-(direction.z * tmp + 1) * 0.5f;
		break;
	case POS_Z: 
		u = (direction.x * tmp + 1) * 0.5f;
		v = (direction.y * tmp + 1) * 0.5f;
		break;
	case NEG_X: 
		u = (direction.z * tmp + 1) * 0.5f;
		v = (direction.y * tmp + 1) * 0.5f;
		break;
	case NEG_Y: 
		u = (direction.x * tmp + 1) * 0.5f;
		v = (direction.z * tmp + 1) * 0.5f;
		break;
	case NEG_Z: 
		u = 1-(direction.x * tmp + 1) * 0.5f;
		v = (direction.y * tmp + 1) * 0.5f;
		break;
	default:
		printf("ERROR: unknown direction %d\n", map);
		break;
	}
	
	Color4 texel = maps_[map]->get_texel(u, v);
	return texel;
}