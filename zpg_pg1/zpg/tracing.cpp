#include "stdafx.h"

//rtcIntersect(*scene, rtc_ray); // find the closest hit of a ray segment with the scene
//rtcOccluded(scene, rtc_ray); // determining if any hit between a ray segment and the scene exists


Tracer::Tracer(const int width, const int height)
{
	width_ = width;
	height_ = height;
	done = false;

	src_32fc3_img = cv::Mat(height, width, CV_32FC3);

	// default fov = 40
	//camera = new Camera(width, height, Vector3(-400.0f, -500.0f, 370.0f), Vector3(70.0f, -40.5f, 5.0f), DEG2RAD(40.0f));
	//camera = new Camera(width, height, Vector3(-200, 150, -300)*0.8, Vector3(100, -50, -50), DEG2RAD(110.0f)); 
	//camera = new Camera(width, height, Vector3(1, 1, 100), Vector3(2, 2, 1000), DEG2RAD(60.0f));
	//camera = new Camera(width, height, Vector3(0, 0, 1), Vector3(1000, 0, 1000), DEG2RAD(90.0f)); 
	camera = new Camera(width, height, Vector3(-400, 150, -300)*0.6, Vector3(100, -50, -50), DEG2RAD(110.0f));
	//camera = new Camera(width, height, Vector3(-400, 370, -500), Vector3(70, 5, -40), DEG2RAD(40.0f));
    camera->Print();

	std::string directory = "../../data/";
	cubemap = new CubeMap(directory);
}

void Tracer::SetScene(RTCScene *scene, std::vector<Surface*> *surfaces)
{
	this->scene = scene;
	this->surfaces = *surfaces;
}

void Tracer::Render()
{
	printf("\nRender started\n");
	std::clock_t timeStart = std::clock();

	Vector3 lightDir = camera->view_from();
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			Ray rtc_ray = camera->GenerateRay(x, y);
			rtcIntersect(*scene, rtc_ray); 

			//src_32fc3_img.at<cv::Vec3f>(y, x) = TraceNormal(rtc_ray);
			src_32fc3_img.at<cv::Vec3f>(y, x) = TraceLambert(rtc_ray);
		}
	}
	
	std::clock_t timeStop = std::clock();
	printf("\nRendered in %d ms\n", (timeStop - timeStart));
	done = true;
}

Vector3 Tracer::GetNormal(Ray ray) {
	Vector3 n = surfaces[ray.geomID]->get_triangle(ray.primID).normal(ray.u, ray.v).Normalized();
	//return n;
	//return -Vector3(n.x, n.y, n.z);
	return ((Vector3)(ray.Ng)).Normalized();
}

cv::Vec3f Tracer::TraceNormal(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 normal = GetNormal(ray);
	Vector3 color = ((normal * 0.5f) + Vector3(0.5f, 0.5f, 0.5f));

	return cv::Vec3f(color.z, color.y, color.x);
}

cv::Vec3f Tracer::TraceLambert(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 color = Vector3(0.5f, 0.5f, 0.5f);
	Vector3 ambient = Vector3(0.1f, 0.1f, 0.1f);

	Vector3 point = ray.eval(ray.tfar);
	Vector3 camPos = camera->view_from();
	Vector3 lightDir = (point - camPos).Normalized();

	Vector3 normal = -GetNormal(ray);

	float dot = normal.DotProduct(lightDir);
	Vector3 lambert = MAX(0, dot) * color;
	return cv::Vec3f(lambert.z, lambert.y, lambert.x);
}

cv::Vec3f Tracer::TracePhong(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 point = ray.eval(ray.tfar);
	Vector3 camPos = camera->view_from();
	Vector3 lightDir = (point - camPos).Normalized();
	Vector3 matColor = surfaces[ray.geomID]->get_material()->diffuse;

	Vector3 normal = GetNormal(ray);
	float dot = normal.DotProduct(lightDir);

	Vector3 ambient = Vector3(0.1f, 0.1f, 0.1f);
	Vector3 diffuse = matColor;
	Vector3 phong = ambient + dot * diffuse;


	Vector3 lambert = dot * Vector3(0.5f, 0.5f, 0.5f);
	return cv::Vec3f(lambert.z, lambert.y, lambert.x);
}

cv::Vec3f Tracer::GetCubeMapColor(Vector3 dir) {
	Color4 c4 = cubemap->get_texel(dir);
	return cv::Vec3f(c4.b, c4.g, c4.r);
}

void Tracer::TestBackgroundRender()
{
	cv::Vec3f c;
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			Ray rtc_ray = camera->GenerateRay(x, y);
			src_32fc3_img.at<cv::Vec3f>(y, x) = GetCubeMapColor(rtc_ray.dir);
		}
	}

	cubemap->PrintShowedTextures();
}

void Tracer::ShowScene() {
	cv::namedWindow("raytracer");
	cv::imshow("raytracer", src_32fc3_img);
}

void Tracer::RenderInThread() {
	m_thread = std::thread(&Tracer::Render, this);
	ShowSceneLoop();
}

void Tracer::ShowSceneLoop() {
	printf("Refreshing ");
	do {
		printf(".");
		ShowScene();
	} while (!done && cv::waitKey(100) != 32);
	m_thread.join();
	printf("\nRefreshing stop\n");
}
