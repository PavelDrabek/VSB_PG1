#include "stdafx.h"

//rtcIntersect(*scene, rtc_ray); // find the closest hit of a ray segment with the scene
//rtcOccluded(scene, rtc_ray); // determining if any hit between a ray segment and the scene exists

cv::Vec3d ToColor(Vector3 v) {
	return cv::Vec3d(v.z, v.y, v.x);
}

Vector3 ToVector(cv::Vec3d c) {
	return Vector3(c.val[2], c.val[1], c.val[0]);
}

Tracer::Tracer(const int width, const int height)
{
	width_ = width;
	height_ = height;
	done = false;
	returnInterrupt = returnFinish = nullDiffuseCount = raysAll = 0;
	for (int i = 0; i < maxDeep; i++) {
		returnCubemap[i] = 0;
	}

	src_32fc3_img = cv::Mat(height, width, CV_64FC3);

	// default fov = 40
	//camera = new Camera(width, height, Vector3(-400.0f, -500.0f, 370.0f), Vector3(70.0f, -40.5f, 5.0f), DEG2RAD(40.0f));
	//camera = new Camera(width, height, Vector3(-200, 150, -300)*0.8, Vector3(100, -50, -50), DEG2RAD(110.0f)); 
	//camera = new Camera(width, height, Vector3(1, 1, 100), Vector3(2, 2, 1000), DEG2RAD(60.0f));
	//camera = new Camera(width, height, Vector3(0, 0, 1), Vector3(1000, 0, 1000), DEG2RAD(90.0f)); 
	//camera = new Camera(width, height, Vector3(-400, 150, -300)*0.6, Vector3(100, -50, -50), DEG2RAD(110.0f));
	camera = new Camera(width, height, Vector3(-400, 370, -500), Vector3(70, 5, -40), DEG2RAD(40.0f));
    camera->Print();

	//lightPos = camera->view_from();
	lightPos = Vector3(-200, 0, 0);

	std::string directory = "../../data/";
	cubemap = new CubeMap(directory);

	Vector3 n = Vector3(0, 1, 0);
	Vector3 d = Vector3(10, 1, 0).Normalized();
	Vector3 v = -n.Reflect(d);
	printf("direction: %f.2, %f.2, %f.2\n", d.x, d.y, d.z);
	printf("reflect:   %f.2, %f.2, %f.2\n", v.x, v.y, v.z);
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
			//rtcIntersect(*scene, rtc_ray); 

			//src_32fc3_img.at<cv::Vec3d>(y, x) = TraceNormal(rtc_ray);
			//src_32fc3_img.at<cv::Vec3d>(y, x) = TraceLambert(rtc_ray);
			src_32fc3_img.at<cv::Vec3d>(y, x) = TracePhong(rtc_ray, 0);
		}
	}
	
	std::clock_t timeStop = std::clock();
	printf("\nRendered in %d ms with %d rays\n", (timeStop - timeStart), raysAll);
	printf("diffuse nulled: %dx\n", nullDiffuseCount);
	printf("return interrupt: %d\n", returnInterrupt);
	printf("return finished: %d\n", returnFinish);
	for (int i = 0; i < maxDeep; i++) {
		printf("return cubemap[%d]: %d\n", i, returnCubemap[i]);
	}
	done = true;
}

cv::Vec3d Tracer::TracePhong(Ray ray, int deep) {
	if (deep >= maxDeep) {
		returnInterrupt++;
		return cv::Vec3d(0, 0, 0);
		//return GetCubeMapColor(ray.dir);
	}

	rtcIntersect(*scene, ray);
	raysAll++;

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		returnCubemap[deep]++;
		return GetCubeMapColor(ray.dir);
	}

	Vector3 viewDir = ray.dir;
	Vector3 point = GetPoint(ray);
	Vector3 normal = GetNormal(ray);
	Vector3 lightDir = GetLightDir(point);
	Vector3 lightReflect = normal.Reflect(lightDir);

	float dotDif = MAX(0, normal.DotProduct(lightDir));
	float dotSpec = MAX(0, viewDir.DotProduct(lightReflect));

	Ray lightRay = Ray(point, lightDir);
	rtcOccluded(*scene, lightRay);
	int visibCoef = 1;
	if (lightRay.geomID == 0) {
		nullDiffuseCount++;
		visibCoef = 0;
	}

	Vector3 ambient = Vector3(0.1f, 0.1f, 0.1f);
	Vector3 diffuse = GetColor(ray);
	//Vector3 diffuse = Vector3(0.5f, 0.5f, 0.5f);
	cv::Vec3d specular = ToColor(diffuse) * TracePhong(Ray(point, normal.Reflect(viewDir)), deep + 1) * dotSpec;
	cv::Vec3d phong = ToColor(ambient) + ToColor(visibCoef * diffuse * dotDif) + specular;

	returnFinish++;
	return phong;
}

cv::Vec3d Tracer::TraceLambert(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 diffuse = GetColor(ray); // Vector3(0.5f, 0.5f, 0.5f);
	Vector3 ambient = Vector3(0.1f, 0.1f, 0.1f);

	float dot = GetNormal(ray).DotProduct(GetLightDir(GetPoint(ray)));
	Vector3 lambert = MAX(0, dot) * diffuse;
	return cv::Vec3d(lambert.z, lambert.y, lambert.x);
}

cv::Vec3d Tracer::TraceNormal(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 normal = GetNormal(ray);
	Vector3 color = ((normal * 0.5f) + Vector3(0.5f, 0.5f, 0.5f));

	return cv::Vec3d(color.z, color.y, color.x);
}

Vector3 Tracer::GetNormal(Ray &ray) {
	Vector3 n = surfaces[ray.geomID]->get_triangle(ray.primID).normal(ray.u, ray.v).Normalized();
	Vector3 n2 = ((Vector3)(ray.Ng)).Normalized();
	//return n2;
	return Vector3(n.x, n.z, n.y);
}

Vector3 Tracer::GetColor(Ray &ray) {
	return surfaces[ray.geomID]->get_material()->diffuse;
}

Vector3 Tracer::GetPoint(Ray &ray) {
	return ray.eval(ray.tfar) - ((Vector3)ray.dir * 0.001f);
}

Vector3 Tracer::GetLightPos() {
	return lightPos;
}

Vector3 Tracer::GetLightDir(Vector3 point) {
	return (GetLightPos() - point).Normalized();
}

cv::Vec3d Tracer::GetCubeMapColor(Vector3 dir) {
	Color4 c4 = cubemap->get_texel(dir);
	return cv::Vec3d(c4.b, c4.g, c4.r);
}

void Tracer::TestBackgroundRender()
{
	cv::Vec3d c;
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			Ray rtc_ray = camera->GenerateRay(x, y);
			src_32fc3_img.at<cv::Vec3d>(y, x) = GetCubeMapColor(rtc_ray.dir);
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
