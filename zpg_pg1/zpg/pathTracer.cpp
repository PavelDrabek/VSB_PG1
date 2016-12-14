#include "stdafx.h"

//rtcIntersect(*scene, rtc_ray); // find the closest hit of a ray segment with the scene
//rtcOccluded(scene, rtc_ray); // determining if any hit between a ray segment and the scene exists

/*
kniha - Global compendium (str 18, bod 34)
### renderovaci rovnice
wo = -viewDir
wi =
Li(wi) ... vstupni tok
fr (x, y) .. fce vracejici scalar intenzity
Lo (wo) = Le(wo) + integral( Li(wi) * fr(wo, wi) * (n * wi) ) d_wi
### nahrazujeme integral sumou (metoda monte carlo)
~~ (b-a) (1/N) suma( fi ) ~~ 1/N suma( (fi(xi) / pdf(xi) )

vygenerovat kouli a rejectnout vuci normale
uniformni, ne normalni!

Li(wi).. ze zacatku 1, potom dosadit cubemapu
fr(x,y) .. lambertuv povrch, jsou nam jedno vstupy, vracime "konstantu" (albedo/PI)
albedo .. <0,1> ... odrazivost povrchu
pokud dlouho nic netrefim, vracim L0 = 0, pripadne cubemapa mi muze vratit 1 nebo velky (plochou) zdroj svetla

*/

cv::Vec3d PathTracer::ToColor(Vector3 v) {
	return cv::Vec3d(v.z, v.y, v.x);
}

Vector3 PathTracer::ToVector(cv::Vec3d c) {
	return Vector3(c.val[2], c.val[1], c.val[0]);
}

PathTracer::PathTracer(const int width, const int height)
{
	width_ = width;
	height_ = height;
	done = false;
	returnInterrupt = returnFinish = nullDiffuseCount = raysAll = 0;
	for (int i = 0; i < maxDeep; i++) {
		returnCubemap[i] = 0;
	}

	src_32fc3_img = cv::Mat(height, width, CV_64FC3);

	camera = new Camera(width, height, Vector3(-140.0f, 110.0f, -175.0f), Vector3(0.0f, 40.0f, 0.0f), DEG2RAD(42.185f));
	camera->Print();

	lightPos = Vector3(-500, 0, 0);

	std::string directory = "../../data/church";
	cubemap = new CubeMap(directory);
}

void PathTracer::SetScene(RTCScene *scene, std::vector<Surface*> *surfaces)
{
	this->scene = scene;
	this->surfaces = *surfaces;
}

void PathTracer::Render()
{
	printf("\nRender started\n");
	std::clock_t timeStart = std::clock();

	Vector3 lightDir = camera->view_from();
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			Ray rtc_ray = camera->GenerateRay(x, y);
			//src_32fc3_img.at<cv::Vec3d>(y, x) = ToColor(TracePhong(rtc_ray, 0));
			src_32fc3_img.at<cv::Vec3d>(y, x) = ToColor(TraceLight(rtc_ray, 0));
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

Vector3 PathTracer::TracePhong(Ray ray, int deep) {
	if (deep >= maxDeep) {
		returnInterrupt++;
		return Vector3(0, 0, 0);
		//return GetCubeMapColor(ray.dir);
	}

	rtcIntersect(*scene, ray);
	raysAll++;

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		returnCubemap[deep]++;
		//return Vector3(1, 1, 1);
		return GetCubeMapColor(ray.dir);
	}

	Surface* surface = surfaces[ray.geomID];
	Triangle triangle = surface->get_triangle(ray.primID);
	Material* material = surface->get_material();
	Texture* tex_diff = material->get_texture(Material::kDiffuseMapSlot);
	float mat_ior = material->ior;
	float transparency = material->transparency;

	Vector3 point = GetPoint(ray);
	Vector3 normal = GetNormal(ray);
	Vector3 viewDir = -Vector3(ray.dir);
	Vector3 lightDir = (lightPos - point).Normalized();
	Vector3 viewReflect = normal.Reflect(lightDir);
	Vector3 lightReflect = normal.Reflect(viewDir);

	Vector3 matColor = material->diffuse;
	if (tex_diff != NULL) {
		Vector2 tuv = triangle.texture_coord(ray.u, ray.v);
		Color4 texel_diff = tex_diff->get_texel(tuv.x, tuv.y);
		matColor = Vector3(texel_diff.r, texel_diff.g, texel_diff.b);
	}

	float dotDif = MAX(0, normal.DotProduct(lightDir));
	float dotSpec = MAX(0, pow(viewDir.DotProduct(lightReflect), 2));
	float cosOoN = MAX(0, normal.DotProduct(viewDir));

	double pdf = M_1_2PI;

	Vector3 ambient = Vector3(1, 1, 1) * 0.1f;
	Vector3 rayOrigin = (Vector3)ray.org;
	Vector3 rayDir = (Vector3)ray.dir;
	Ray lightRay = Ray(rayOrigin, rayDir);
	Vector3 light = TraceLight(lightRay, 0);
	Vector3 phong = ambient + light * cosOoN * matColor;

	returnFinish++;
	return light;
}

/*
Dalsi body za:
* cosinus vazene vzorkovani (compendium 35)
* BRDF zrcadla
*/

Vector3 PathTracer::TraceLight(Ray ray, int deep) {
	if (deep >= maxDeep) {
		return Vector3(0, 0, 0);
	}

	rtcIntersect(*scene, ray);
	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
		//return Vector3(1, 1, 1);
	}

	double pdf = M_1_2PI;
	double M_1_pdf = M_2PI;
	Vector3 point = GetPoint(ray);
	Vector3 normal = GetNormal(ray);

	int nrays = (deep > 0) ? 1 : 40;
	
	//Vector3 albedo = GetColor(ray);
	Vector3 albedo = Vector3(0.6f);
	Vector3 fr = albedo / M_PI;
	Vector3 wo = -Vector3(ray.dir);	// outgoing to camera
	//Vector3 Le = Vector3(0.0f); // GetColor(ray);
	
	Vector3 Lo = Vector3(0.0f);
	for (int i = 0; i < nrays; i++)
	{
		//Vector3 wi = GetOmega(normal);	// incoming from light
		Vector3 wi = GetOmegaReflection(normal, wo, 0.01f);	// incoming from light
		float dot = normal.DotProduct(wi);
		Ray wRay = Ray(point, wi);
		Vector3 Li = TraceLight(wRay, deep + 1) * M_1_pdf;
		Lo += /*Le + */Li * fr * dot;
	}
	Lo /= nrays;

	return Lo;
}

Vector3 PathTracer::GetOmegaReflection(Vector3 normal, Vector3 incoming, float blur) {
	Vector3 omega = normal.Reflect(-incoming);

	double r1 = Random(0, 1);
	double r2 = Random(0, 1);
	double phi = M_2PI * r1;

	float sqrt_1r = std::sqrt(1 - r2 * r2);
	Vector3 rnd = Vector3(cos(phi) * sqrt_1r, sin(phi) * sqrt_1r, r2).Normalized(); 
	rnd = (rnd.DotProduct(normal) < 0) ? -rnd : rnd;

	omega = omega + ((rnd - omega) * blur);
	return omega.Normalized();
}

Vector3 PathTracer::GetOmega(Vector3 normal) {
	double r1 = Random(0, 1);
	double r2 = Random(0, 1);
	double phi = M_2PI * r1;

	float sqrt_1r = std::sqrt(1 - r2 * r2);
	Vector3 omega = Vector3(cos(phi) * sqrt_1r, sin(phi) * sqrt_1r, r2).Normalized();
	return (omega.DotProduct(normal) < 0) ? -omega : omega;
}

Vector3 PathTracer::TraceLambert(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 diffuse = GetColor(ray); // Vector3(0.5f, 0.5f, 0.5f);
	Vector3 ambient = Vector3(0.1f, 0.1f, 0.1f);

	float dot = GetNormal(ray).DotProduct(GetLightDir(GetPoint(ray)));
	Vector3 lambert = MAX(0, dot) * diffuse;
	return lambert;
}

Vector3 PathTracer::TraceNormal(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 normal = GetNormal(ray);
	Vector3 color = ((normal * 0.5f) + Vector3(0.5f, 0.5f, 0.5f));

	return color;
}

Vector3 PathTracer::GetNormal(Ray &ray) {
	Vector3 n = surfaces[ray.geomID]->get_triangle(ray.primID).normal(ray.u, ray.v).Normalized();
	n = Vector3(n.x, n.z, n.y);
	if (n.DotProduct(ray.dir) > 0) // neni < 0, protoze paprsek leti opacnym smerem, tak at ho nemusim otacet
	{ 
		n = -n;
	}
	Vector3 n2 = ((Vector3)(ray.Ng)).Normalized();
	//return n2;
	return n;
}

Vector3 PathTracer::GetColor(Ray &ray) {
	return surfaces[ray.geomID]->get_material()->diffuse;
}

Vector3 PathTracer::GetPoint(Ray &ray, bool stepBack) {
	return ray.eval(ray.tfar) - ((Vector3)ray.dir * 0.001f) * (stepBack ? 1 : -1);
}

Vector3 PathTracer::GetLightPos() {
	return lightPos;
}

Vector3 PathTracer::GetLightDir(Vector3 point) {
	return (GetLightPos() - point).Normalized();
}

Vector3 PathTracer::GetCubeMapColor(Vector3 dir) {
	Color4 c4 = cubemap->get_texel(dir);
	return Vector3(c4.r, c4.g, c4.b);
}

void PathTracer::TestBackgroundRender()
{
	cv::Vec3d c;
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			Ray rtc_ray = camera->GenerateRay(x, y);
			src_32fc3_img.at<cv::Vec3d>(y, x) = ToColor(GetCubeMapColor(rtc_ray.dir));
		}
	}

	cubemap->PrintShowedTextures();
}

void PathTracer::ShowScene() {
	cv::namedWindow("path tracer");
	cv::setMouseCallback("path tracer", onMouse, this);
	cv::imshow("path tracer", src_32fc3_img);
}

void PathTracer::RenderInThread() {
	m_thread = std::thread(&PathTracer::Render, this);
	ShowSceneLoop();
}

void PathTracer::ShowSceneLoop() {
	printf("Refreshing ");
	do {
		printf(".");
		ShowScene();
	} while (!done && cv::waitKey(100) != 32);
	m_thread.join();
	printf("\nRefreshing stop\n");
}

void PathTracer::onMouse(int event, int x, int y, int flags, void* userdata)
{
	if (event != CV_EVENT_LBUTTONDOWN)
		return;

	PathTracer* tracer = reinterpret_cast<PathTracer*>(userdata);
	cv::Vec3d c = tracer->src_32fc3_img.at<cv::Vec3d>(y, x);

	printf("Debug for y = %d, x = %d\n", y, x);
	printf("  color = (%d, %d, %d)\n", (int)(c.val[2] * 255), (int)(c.val[1] * 255), (int)(c.val[0] * 255));
	//std::cout << "y=" << y << "\t x=" << x << "\t value=" << c << "\n";



	Ray ray = tracer->camera->GenerateRay(x, y);

	rtcIntersect(*(tracer->scene), ray);
	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return;
	}

	Vector3 point = tracer->GetPoint(ray);
	Vector3 normal = tracer->GetNormal(ray);

	//int nrays = 50;
	//if (deep > 0) {
	//	nrays = 1;
	//}

	double pdf = M_1_2PI;
	Vector3 omega = tracer->GetOmega(normal);
	printf("  normal = (%.2f, %.2f, %.2f)\n", normal.x, normal.y, normal.z);
	printf("  omega  = (%.2f, %.2f, %.2f)\n", omega.x, omega.y, omega.z);
	printf("  n . o  = %.4f\n", normal.DotProduct(omega));
	Ray wRay = Ray(point, omega);
	Vector3 light = tracer->TraceLight(wRay, maxDeep - 1);
	printf("  light  = (%.2f, %.2f, %.2f)\n", light.x, light.y, light.z);

	//Vector3 light = Vector3(0, 0, 0);
	//for (int i = 0; i < nrays; i++)
	//{
	//	Vector3 omega = GetOmega(normal);
	//	Ray wRay = Ray(point, omega);
	//	light += TraceLight(wRay, deep + 1);
	//}
}