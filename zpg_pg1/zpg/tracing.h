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

	cv::Mat src_32fc3_img;

private:

	int width_; /*!< Šíøka obrazu [px]. */
	int height_; /*!< Výška obrazu [px]. */
	bool done;
	std::thread t1;

	Camera* camera;
	RTCScene* scene;
	CubeMap* cubemap;
	std::vector<Surface*> surfaces;

	Vector3 GetNormal(Ray ray);
	cv::Vec3f GetCubeMapColor(Vector3 dir);
	
	cv::Vec3f TraceNormal(Ray ray);
	cv::Vec3f TraceLambert(Ray ray);
	cv::Vec3f TracePhong(Ray ray);

	std::thread m_thread;
};
