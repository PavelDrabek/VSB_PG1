#pragma once
#include <thread>

#define DEG2RAD( x ) ( ( x ) * static_cast<float>( M_PI / 180 ) )

/*
 * author Pavel Drabek
 * created  5.10.2016
*/
class Tracer
{
public:
	Tracer(const int width, const int height);
	void Render();
	void RenderInThread();
	void TestBackgroundRender();

	void SetScene(RTCScene *scene, std::vector<Surface*> *surfaces);
	void ShowScene();
	void ShowSceneLoop();

	static void onMouse(int event, int x, int y, int, void*);

	cv::Mat src_32fc3_img;

private:

	static const int maxDeep = 5;

	int width_; /*!< Šíøka obrazu [px]. */
	int height_; /*!< Výška obrazu [px]. */
	bool done;

	int nullDiffuseCount;
	int raysAll;
	int returnInterrupt;
	int returnFinish;
	int returnCubemap[maxDeep];

	Camera* camera;
	RTCScene* scene;
	CubeMap* cubemap;
	std::vector<Surface*> surfaces;

	Vector3 lightPos;

	Vector3 GetPoint(Ray &ray);
	Vector3 GetNormal(Ray &ray);
	Vector3 GetLightPos();
	Vector3 GetLightDir(Vector3 point);
	Vector3 GetColor(Ray &ray);

	cv::Vec3d GetCubeMapColor(Vector3 dir);

	cv::Vec3d TraceNormal(Ray ray);
	cv::Vec3d TraceLambert(Ray ray);
	cv::Vec3d TracePhong(Ray ray, int deep);

	std::thread m_thread;
};
