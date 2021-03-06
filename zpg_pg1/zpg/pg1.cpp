#include "stdafx.h"

void rtc_error_function( const RTCError code, const char * str )
{
	printf( "ERROR in Embree: %s\n", str );
	exit( 1 );
}

RTCError check_rtc_or_die( RTCDevice & device )
{
	const RTCError error = rtcDeviceGetError( device );

	if ( error != RTC_NO_ERROR )
	{
		printf( "ERROR in Embree: " );

		switch ( error )
		{
		case RTC_UNKNOWN_ERROR:
			printf( "An unknown error has occurred." );
			break;

		case RTC_INVALID_ARGUMENT:
			printf( "An invalid argument was specified." );
			break;

		case RTC_INVALID_OPERATION:
			printf( "The operation is not allowed for the specified object." );
			break;

		case RTC_OUT_OF_MEMORY:
			printf( "There is not enough memory left to complete the operation." );
			break;

		case RTC_UNSUPPORTED_CPU:
			printf( "The CPU is not supported as it does not support SSE2." );
			break;

		case RTC_CANCELLED:
			printf( "The operation got cancelled by an Memory Monitor Callback or Progress Monitor Callback function." );
			break;
		}

		fflush( stdout );
		exit( 1 );
	}

	return error;
}

// struktury pro ukládání dat pro Embree
namespace embree_structs
{
	struct Vertex { float x, y, z, a; };
	typedef Vertex Normal;
	struct Triangle { int v0, v1, v2; };
};

void filter_intersection( void * user_ptr, Ray & ray )
{
	/*  All hit information inside the ray is valid.
		The filter function can reject a hit by setting the geomID member of the ray to
	    RTC_INVALID_GEOMETRY_ID, otherwise the hit is accepted.The filter function is not
		allowed to modify the ray input data (org, dir, tnear, tfar), but can modify
		the hit data of the ray( u, v, Ng, geomID, primID ). */

	Surface * surface = reinterpret_cast<Surface *>( user_ptr );	
	printf( "intersection of: %s, ", surface->get_name().c_str() );
	const Vector3 p = ray.eval( ray.tfar );
	printf( "at: %0.3f (%0.3f, %0.3f, %0.3f)\n", ray.tfar, p.x, p.y, p.z );
	
	ray.geomID = RTC_INVALID_GEOMETRY_ID; // reject hit
}

void filter_occlusion( void * user_ptr, Ray & ray )
{
	/*  All hit information inside the ray is valid.
	The filter function can reject a hit by setting the geomID member of the ray to
	RTC_INVALID_GEOMETRY_ID, otherwise the hit is accepted.The filter function is not
	allowed to modify the ray input data (org, dir, tnear, tfar), but can modify
	the hit data of the ray( u, v, Ng, geomID, primID ). */

	Surface * surface = reinterpret_cast<Surface *>( user_ptr );	
	printf( "occlusion of: %s, ", surface->get_name().c_str() );
	const Vector3 p = ray.eval( ray.tfar );
	printf( "at: %0.3f (%0.3f, %0.3f, %0.3f)\n", ray.tfar, p.x, p.y, p.z );

	ray.geomID = RTC_INVALID_GEOMETRY_ID; // reject hit
}

int test( RTCScene & scene, std::vector<Surface *> & surfaces )
{
	// --- test rtcIntersect -----
	//Ray rtc_ray = Ray(Vector3(-1.0f, 0.1f, 0.2f), Vector3(2.0f, 0.0f, 0.0f), 0.0f);
	Ray rtc_ray = Ray( Vector3( -1.0f, 0.1f, 0.2f ), Vector3( 2.0f, 0.0f, 0.0f ), 0.0f );
	//Ray rtc_ray = Ray( Vector3( 4.0f, 0.1f, 0.2f ), Vector3( -1.0f, 0.0f, 0.0f ) );
	//rtc_ray.tnear = 0.6f;
	//rtc_ray.tfar = 2.0f;
	rtcIntersect( scene, rtc_ray ); // find the closest hit of a ray segment with the scene
	// pri filtrovani funkce rtcIntersect jsou pruseciky prochazeny od nejblizsiho k nejvzdalenejsimu
	// u funkce rtcOccluded jsou nalezene pruseciky vraceny v libovolnem poradi

	if ( rtc_ray.geomID != RTC_INVALID_GEOMETRY_ID )
	{
		Surface * surface = surfaces[rtc_ray.geomID];
		Triangle & triangle = surface->get_triangle( rtc_ray.primID );
		//Triangle * triangle2 = &( surface->get_triangles()[rtc_ray.primID] );

		// získání souřadnic průsečíku, normál, texturovacích souřadnic atd.
		const Vector3 p = rtc_ray.eval( rtc_ray.tfar );
		Vector3 geometry_normal = -Vector3( rtc_ray.Ng ); // Ng je nenormalizovaná normála zasaženého trojúhelníka vypočtená nesouhlasně s pravidlem pravé ruky o závitu
		geometry_normal.Normalize(); // normála zasaženého trojúhelníka vypočtená souhlasně s pravidlem pravé ruky o závitu
		const Vector3 normal = triangle.normal( rtc_ray.u, rtc_ray.v );
		const Vector2 texture_coord = triangle.texture_coord( rtc_ray.u, rtc_ray.v );

		printf("");
	}

	// --- test rtcOccluded -----
	rtc_ray = Ray( Vector3( -1.0f, 0.1f, 0.2f ), Vector3( 1.0f, 0.0f, 0.0f ) );
	//rtc_ray.tfar = 1.5;	
	rtcOccluded( scene, rtc_ray ); // determining if any hit between a ray segment and the scene exists
	// po volání rtcOccluded je nastavena pouze hodnota geomID, ostatni jsou nezměněny
	if ( rtc_ray.geomID == 0 )
	{
		// neco jsme nekde na zadaném intervalu (tnear, tfar) trefili, ale nevime co ani kde
	}

	return 0;
}

/*
Seznam úkolů:

1, Doplnit TODO v souboru tracing.cpp.
*/

int main( int argc, char * argv[] )
{
	printf( "PG1, (c)2016 Pavel Drabek, DRA0042\n\n" );	

	_MM_SET_FLUSH_ZERO_MODE( _MM_FLUSH_ZERO_ON ); // Flush to Zero, Denormals are Zero mode of the MXCSR
	_MM_SET_DENORMALS_ZERO_MODE( _MM_DENORMALS_ZERO_ON );
	RTCDevice device = rtcNewDevice( NULL ); // musíme vytvořit alespoň jedno Embree zařízení		
	check_rtc_or_die( device ); // ověření úspěšného vytvoření Embree zařízení
	rtcDeviceSetErrorFunction( device, rtc_error_function ); // registrace call-back funkce pro zachytávání chyb v Embree	

	std::vector<Surface *> surfaces;
	std::vector<Material *> materials;

	// načtení geometrie
	std::string modelName = "6887_allied_avenger.obj";
	//std::string modelName = "geosphere.obj";
	std::string modelPath = "../../data/" + modelName;
	if (LoadOBJ(modelPath.c_str(), Vector3( 0.5f, 0.5f, 0.5f ), surfaces, materials, true ) < 0 )
	{
		return -1;
	}
	
	// vytvoření scény v rámci Embree
	RTCScene scene = rtcDeviceNewScene( device, RTC_SCENE_STATIC | RTC_SCENE_HIGH_QUALITY, RTC_INTERSECT1/* | RTC_INTERPOLATE*/ );
	// RTC_INTERSECT1 = enables the rtcIntersect and rtcOccluded functions

	// nakopírování všech modelů do bufferů Embree
	for ( std::vector<Surface *>::const_iterator iter = surfaces.begin();
		iter != surfaces.end(); ++iter )
	{
		Surface * surface = *iter;
		unsigned geom_id = rtcNewTriangleMesh( scene, RTC_GEOMETRY_STATIC,
			surface->no_triangles(), surface->no_vertices() );

		//rtcSetUserData, rtcSetBoundsFunction, rtcSetIntersectFunction, rtcSetOccludedFunction,
		rtcSetUserData( scene, geom_id, surface );
		//rtcSetOcclusionFilterFunction, rtcSetIntersectionFilterFunction		
		//rtcSetOcclusionFilterFunction( scene, geom_id, reinterpret_cast< RTCFilterFunc >( &filter_occlusion ) );
		//rtcSetIntersectionFilterFunction( scene, geom_id, reinterpret_cast< RTCFilterFunc >( &filter_intersection ) );

		// kopírování samotných vertexů trojúhelníků
		embree_structs::Vertex * vertices = static_cast< embree_structs::Vertex * >(
			rtcMapBuffer( scene, geom_id, RTC_VERTEX_BUFFER ) );

		for ( int t = 0; t < surface->no_triangles(); ++t )
		{
			for ( int v = 0; v < 3; ++v )
			{
				embree_structs::Vertex & vertex = vertices[t * 3 + v];

				vertex.x = surface->get_triangles()[t].vertex( v ).position.x;
				vertex.y = surface->get_triangles()[t].vertex( v ).position.y;
				vertex.z = surface->get_triangles()[t].vertex( v ).position.z;
			}
		}

		rtcUnmapBuffer( scene, geom_id, RTC_VERTEX_BUFFER );

		// vytváření indexů vrcholů pro jednotlivé trojúhelníky
		embree_structs::Triangle * triangles = static_cast< embree_structs::Triangle * >(
			rtcMapBuffer( scene, geom_id, RTC_INDEX_BUFFER ) );

		for ( int t = 0, v = 0; t < surface->no_triangles(); ++t )
		{
			embree_structs::Triangle & triangle = triangles[t];

			triangle.v0 = v++;
			triangle.v1 = v++;
			triangle.v2 = v++;
		}

		rtcUnmapBuffer( scene, geom_id, RTC_INDEX_BUFFER );

		/*embree_structs::Normal * normals = static_cast< embree_structs::Normal * >(
			rtcMapBuffer( scene, geom_id, RTC_USER_VERTEX_BUFFER0 ) );
			rtcUnmapBuffer( scene, geom_id, RTC_USER_VERTEX_BUFFER0 );*/
	}

	rtcCommit( scene );	

	//test( scene, surfaces );

	//PathTracer tracer = PathTracer(640, 480);
	Tracer tracer = Tracer(640, 480);
	tracer.SetScene(&scene, &surfaces);

	tracer.RenderInThread();
	//tracer.Render();
	//tracer.ShowScene(); 
	cv::waitKey(0);

	rtcDeleteScene( scene ); // zrušení Embree scény

	SafeDeleteVectorItems<Material *>( materials );
	SafeDeleteVectorItems<Surface *>( surfaces );

	rtcDeleteDevice( device ); // Embree zařízení musíme také uvolnit před ukončením aplikace

	return 0;
}
