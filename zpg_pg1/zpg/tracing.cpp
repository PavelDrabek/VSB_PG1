#include "stdafx.h"

//rtcIntersect(*scene, rtc_ray); // find the closest hit of a ray segment with the scene
//rtcOccluded(scene, rtc_ray); // determining if any hit between a ray segment and the scene exists

cv::Vec3d Tracer::ToColor(Vector3 v) {
	return cv::Vec3d(v.z, v.y, v.x);
}

Vector3 Tracer::ToVector(cv::Vec3d c) {
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
	//camera = new Camera(width, height, Vector3(-400, 370, -500), Vector3(70, 5, -40), DEG2RAD(40.0f));
	//camera = new Camera(width, height, Vector3(-140.0f, 110.0f, -175.0f), Vector3(0.0f, 40.0f, 0.0f), DEG2RAD(42.185f));
	//camera = new Camera(width, height, Vector3(0.0f, 0.0f, -3.0f), Vector3(0.0f, 0.0f, 1.0f), DEG2RAD(42.185f));
	camera = new Camera(width, height, Vector3(0.0f, 0.0f, 3.0f), Vector3(0.0f, 0.0f, -1.0f), DEG2RAD(42.185f));
    camera->Print();

	//lightPos = camera->view_from();
	lightPos = Vector3(-500, 0, 0);

	std::string directory = "../../data/church";
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
//#pragma omp parallel for schedule(dynamic, 5) shared(src_32fc3_img, scene, surfaces)
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			Ray rtc_ray = camera->GenerateRay(x, y);
			//rtcIntersect(*scene, rtc_ray); 

			//src_32fc3_img.at<cv::Vec3d>(y, x) = TraceNormal(rtc_ray);
			//src_32fc3_img.at<cv::Vec3d>(y, x) = TraceLambert(rtc_ray);
			src_32fc3_img.at<cv::Vec3d>(y, x) = ToColor(TracePhong(rtc_ray, 0));
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

Vector3 Tracer::TracePhong(Ray ray, int deep) {
	if (deep >= maxDeep) {
		returnInterrupt++;
		return Vector3(0, 0, 0);
		//return GetCubeMapColor(ray.dir);
	}

	rtcIntersect(*scene, ray);
	raysAll++;

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		returnCubemap[deep]++;
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
	Vector3 viewDir = ray.dir;
	Vector3 lightDir = (lightPos - point).Normalized();
	Vector3 viewReflect = normal.Reflect(lightDir);
	Vector3 lightReflect = normal.Reflect(viewDir);

	Vector2 tuv = triangle.texture_coord(ray.u, ray.v);

	float dotDif = MAX(0, normal.DotProduct(lightDir));
	float dotSpec = MAX(0, pow(viewDir.DotProduct(lightReflect), 2));

	Ray lightRay = Ray(point, lightDir, 0, (GetLightPos() - point).SqrL2Norm());
	rtcOccluded(*scene, lightRay);
	int visibCoef = 1;
	if (lightRay.geomID != -1) {
		nullDiffuseCount++;
		visibCoef = 0;
	}

	// snell law (index lomu - kam se paprsek odrazi), fresnel equation (pomer mezi propustnosti a odrazem)
	// TODO: prekladova tabulka material -> index lomu
	// normala muze byt v modelu otocena spatne - zjistovat, jestli jde paprsek ze vzduchu a pokud je dot(n,ray) zaporny, je otocena
	Vector3 ambient = Vector3(0.1f, 0.1f, 0.1f);
	Vector3 diffuse = material->diffuse;
	if (tex_diff != NULL) {
		Color4 texel_diff = tex_diff->get_texel(tuv.x, tuv.y);
		diffuse = Vector3(texel_diff.r, texel_diff.g, texel_diff.b);
	}

	Vector3 reflected = TracePhong(Ray(point, lightReflect), deep + 1);
	Vector3 retracted = Vector3(0, 0, 0);
	float R = 1;
	float T = 0;
	if (transparency < 1) {
		float n1 = ray.ior;
		float n2 = ray.ior == 1 ? mat_ior : 1;
		Vector3 l = viewDir;
		float c = l.DotProduct(-normal);
		if (c < 0) {
			c = 1;
		}
		float r = n2 / n1;
		float cos_O2 = sqrt(1 - r*r * (1 - c*c));
		Vector3 retractDir = r * l + (r * c - sqrt(1 - r*r * (1 - c*c))) * normal;
		//Vector3 retractDir = r * l + (c - r * cos_O2) * normal;
		Ray retracted_ray = Ray(GetPoint(ray, false), retractDir);
		retracted_ray.ior = n2;
		retracted = TracePhong(retracted_ray, deep + 1);

		float cosi = c;
		float cost = abs(retractDir.DotProduct(normal));
		float n1cosi = n1 * cosi;
		float n1cost = n1 * cost;
		float n2cosi = n2 * cosi;
		float n2cost = n2 * cost;

		float Rs = pow((n1cosi - n2cost) / (n1cosi + n2cost), 2);
		float Rp = pow((n1cost - n2cosi) / (n1cost + n2cosi), 2);
		R = (Rs + Rp) * 0.5f;
		T = 1 - R;

		return reflected * R * (dotSpec * material->specular) + retracted * T * material->diffuse;
	}

	Vector3 specular = (R * reflected * dotSpec * material->specular * material->reflectivity) + T * retracted;
	Vector3 phong = ambient + (diffuse * dotDif * visibCoef) + specular;

	//cv::Vec3d P = ToColor(ambient) + 

	returnFinish++;
	return phong;
}

Vector3 Tracer::TraceLambert(Ray ray) {
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

Vector3 Tracer::TraceNormal(Ray ray) {
	rtcIntersect(*scene, ray);

	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return GetCubeMapColor(ray.dir);
	}

	Vector3 normal = GetNormal(ray);
	Vector3 color = ((normal * 0.5f) + Vector3(0.5f, 0.5f, 0.5f));

	return color;
}

Vector3 Tracer::GetNormal(Ray &ray) {
	Vector3 n = surfaces[ray.geomID]->get_triangle(ray.primID).normal(ray.u, ray.v).Normalized();
	n = Vector3(n.x, n.z, n.y);
	if (n.DotProduct(ray.dir) < 0) // neni < 0, protoze paprsek leti opacnym smerem, tak at ho nemusim otacet
	{
		n = -n;
	}
	Vector3 n2 = ((Vector3)(ray.Ng)).Normalized();
	//return n2;
	return n;
}

Vector3 Tracer::GetColor(Ray &ray) {
	return surfaces[ray.geomID]->get_material()->diffuse;
}

Vector3 Tracer::GetPoint(Ray &ray, bool stepBack) {
	return ray.eval(ray.tfar) - ((Vector3)ray.dir * 0.001f) * (stepBack ? 1 : -1);
}

Vector3 Tracer::GetLightPos() {
	return lightPos;
}

Vector3 Tracer::GetLightDir(Vector3 point) {
	return (GetLightPos() - point).Normalized();
}

Vector3 Tracer::GetCubeMapColor(Vector3 dir) {
	Color4 c4 = cubemap->get_texel(dir);
	return Vector3(c4.r, c4.g, c4.b);
}

void Tracer::TestBackgroundRender()
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

void Tracer::ShowScene() {
	cv::namedWindow("raytracer");
	cv::setMouseCallback("raytracer", onMouse, this);
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

void Tracer::onMouse(int event, int x, int y, int flags, void* userdata)
{
	if (event != CV_EVENT_LBUTTONDOWN)
		return;

	Tracer* tracer = reinterpret_cast<Tracer*>(userdata);
	cv::Vec3d c = tracer->src_32fc3_img.at<cv::Vec3d>(y, x);

	printf("Debug for y = %d, x = %d\n", y, x);
	printf("  color = (%d, %d, %d)\n", (int)(c.val[2] * 255), (int)(c.val[1] * 255), (int)(c.val[0] * 255));

	Ray ray = tracer->camera->GenerateRay(x, y);

	rtcIntersect(*(tracer->scene), ray);
	if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
		return;
	}

	Vector3 point = tracer->GetPoint(ray);
	Vector3 normal = tracer->GetNormal(ray);

	printf("  normal = (%.2f, %.2f, %.2f)\n", normal.x, normal.y, normal.z);

	Surface* surface = tracer->surfaces[ray.geomID];
	Triangle triangle = surface->get_triangle(ray.primID);
	Material* material = surface->get_material();
	Texture* tex_diff = material->get_texture(Material::kDiffuseMapSlot);
	float mat_ior = material->ior;
	float transparency = material->transparency;

	Vector3 viewDir = ray.dir;


	float R = 1;
	float T = 0;
	if (transparency < 1) {
		float n1 = ray.ior;
		float n2 = ray.ior == 1 ? mat_ior : 1;
		Vector3 l = viewDir;
		float c = l.DotProduct(-normal);
		printf("  c = %f\n", c);
		if (c < 0) {
			c = 1;
		}
		float r = n2 / n1;
		float cos_O2 = sqrt(1 - r*r * (1 - c*c));
		Vector3 retractDir = r * l + (r * c - sqrt(1 - r*r * (1 - c*c))) * normal;
		//Vector3 retractDir = r * l + (c - r * cos_O2) * normal;

		float cosi = c;
		float cost = abs(retractDir.DotProduct(-normal));
		float n1cosi = n1 * cosi;
		float n1cost = n1 * cost;
		float n2cosi = n2 * cosi;
		float n2cost = n2 * cost;

		float Rs = pow((n1cosi - n2cost) / (n1cosi + n2cost), 2);
		float Rp = pow((n1cost - n2cosi) / (n1cost + n2cosi), 2);
		R = (Rs + Rp) * 0.5f;
		T = 1 - R;
		printf("  R = %f\n", R);
		printf("  T = %f\n", T);

		//return reflected * R * (dotSpec * material->specular) + retracted * T * material->diffuse;
	}
}