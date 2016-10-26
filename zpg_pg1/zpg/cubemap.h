#pragma once

class CubeMap
{
public:

	enum MAP { POS_X, POS_Y, POS_Z, NEG_X, NEG_Y, NEG_Z };

	CubeMap(std::string directory);
	~CubeMap();

	bool showed[6];
	Texture* maps_[6];
	Color4 get_texel(Vector3 &direction);

	void PrintShowedTextures();

};