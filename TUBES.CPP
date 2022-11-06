#include "tdl.h"
#include "tinymath.h"
#include "vector3.hpp"
#include "pose3.hpp"
#include "htmat4.hpp"
#include "interp.hpp"

#define kDisplayWidth	320
#define kDisplayHeight	200
#define kDisplayBpp		32

volatile bool loop = true;

int EscHandler() {
	loop = false;
	return 0;
}

void CalcPolyNormal(RENDERPOLY &p) {
	POINT3D diff2_1;	
	diff2_1.x = p.P2->PointPos.x - p.P1->PointPos.x;
	diff2_1.y = p.P2->PointPos.y - p.P1->PointPos.y;
	diff2_1.z = p.P2->PointPos.z - p.P1->PointPos.z;
	
	POINT3D diff3_1;
	diff3_1.x = p.P3->PointPos.x - p.P1->PointPos.x;
	diff3_1.y = p.P3->PointPos.y - p.P1->PointPos.y;
	diff3_1.z = p.P3->PointPos.z - p.P1->PointPos.z;

	// Cross product.
	p.PNormal.x = diff3_1.y * diff2_1.z - diff3_1.z * diff2_1.y;
	p.PNormal.y = diff3_1.z * diff2_1.x - diff3_1.x * diff2_1.z;
	p.PNormal.z = diff3_1.x * diff2_1.y - diff3_1.y * diff2_1.x;
	
	float norm = sqrt(p.PNormal.x * p.PNormal.x + p.PNormal.y * p.PNormal.y + p.PNormal.z * p.PNormal.z);
	p.PNormal.x /= norm;
	p.PNormal.y /= norm;
	p.PNormal.z /= norm;
	
	if (p.PolyFlags & kFrontFaceCulled) {
		// Reverse normal
		p.PNormal.x = -p.PNormal.x;
		p.PNormal.y = -p.PNormal.y;
		p.PNormal.z = -p.PNormal.z;
	}
	
	p.OrigPNormal = p.PNormal;
}

int main() {
	int error = 0;
	add_key(&EscHandler, Key_ESC);

	// Generate the mesh by moving a template shape along an interpolated pose trajectory.
	Pose3Trajectory pose_trajectory;
	pose_trajectory.addKeyPoint(0.0f, Pose3(Vector3(0, -1.3, 0), Quaternion(Vector3(0.5, 0, 0.5), -0.5*M_PI)));
	pose_trajectory.addKeyPoint(0.25f, Pose3(Vector3(0.5, -1.3+0.75, 0.25), Quaternion(Vector3(0.5, 0, 0.5), -0.4*M_PI)));
	pose_trajectory.addKeyPoint(0.5f, Pose3(Vector3(1, -1.3+1, 1), Quaternion(Vector3(1, 0, 0), 0)));
	pose_trajectory.addKeyPoint(0.75f, Pose3(Vector3(0.5, -1.3+0.75, 1.75), Quaternion(Vector3(0.5, 0, -0.5), 0.15*M_PI)));

	pose_trajectory.addKeyPoint(1, Pose3(Vector3(0, -1.3, 2), Quaternion(Vector3(1, 0, 0), 0.5*M_PI)));
	pose_trajectory.addKeyPoint(1.25f, Pose3(Vector3(0.5, -1.3-0.75, 2-0.5), Quaternion(Vector3(0.75, 0, 0.25), 0.8*M_PI)));
	pose_trajectory.addKeyPoint(1.75f, Pose3(Vector3(0.9, -1.3-1.25, 0.15), Quaternion(Vector3(0.75, 0, -0.25), M_PI)));
	pose_trajectory.addKeyPoint(2.75f, Pose3(Vector3(-0.25, -1.3-0.25, -1), Quaternion(Vector3(1, 0, 0), -0.5*M_PI)));
	pose_trajectory.addKeyPoint(3.25f, Pose3(Vector3(-1.5, -1.3+0.5, -0.5), Quaternion(Vector3(1, 0, 0), 0*M_PI)));
	pose_trajectory.addKeyPoint(3.75f, Pose3(Vector3(-2, -1.3+0.5, 0.8), Quaternion(Vector3(1, 0, 0), 0.25*M_PI)));
	pose_trajectory.addKeyPoint(4.25f, Pose3(Vector3(-2, -1.3-1, 1.4), Quaternion(Vector3(1, 0, 0), 0.5*M_PI)));
	
	pose_trajectory.addKeyPoint(4.75f, Pose3(Vector3(-1, -1.3-2, 1.5), Quaternion(Vector3(0.75, 0, 0.25), M_PI)));
	pose_trajectory.addKeyPoint(5.25f, Pose3(Vector3(-0.5, -1.3-1, 0.3), Quaternion(Vector3(0.75, 0, 0.25), 1.25*M_PI)));
	pose_trajectory.addKeyPoint(5.5, Pose3(Vector3(0, -1.3, 0), Quaternion(Vector3(0.5, 0, 0.5), -0.5*M_PI)));

	Pose3CubicCatmullRomInterpolator pose_interpolator;
	pose_interpolator.setTrajectory(pose_trajectory);
	
	float trajectory_start_t = pose_trajectory.cbegin()->time();
	float trajectory_end_t = (--pose_trajectory.cend())->time();
	float trajectory_length = trajectory_end_t - trajectory_start_t;
	const int kNumSections = 80;
	const float kSectionRadius = 0.2f;
	const int kNumSectionPoints = 5;
	const float kTextureFactorX = 0.25;
	// const float kTextureFactorY = kNumSections / 5.0f;
	const float kTextureHeightMeters = 4;
	
	if (kNumSections < 2) {
		printf("kNumSections must be >= 3.");
		goto error_exit;
	}

	SLI *texture = LoadGFX("texture.pcx", 8);
	if (texture == NULL) {
		error = -1;
		goto error_exit;
	}
	CreateLitTableSLI(texture, BLACK2WHITE, 129);

	const int num_points = kNumSections * kNumSectionPoints;
	RENDERPOINT *points = new RENDERPOINT[num_points];
	int i = 0;
	Vector3 last_section_center = pose_interpolator.getValue(0).position();
	float distance_over_curve = 0;
	for (int n = 0; n < kNumSections; ++n)  {
		Pose3 pose = pose_interpolator.getValue(trajectory_start_t + (trajectory_length * n) / (kNumSections - 1));
		HTMatrix4 htm(pose.orientation().getRotMatrix(), pose.position());
		distance_over_curve += (pose.position() - last_section_center).modulus();
		// Sample the shape to create points around the current pose.
		for (int a = 0; a < kNumSectionPoints; ++a) {
			float angle = -M_PI + (a * 2 * M_PI) / kNumSectionPoints;
			Vector4 point = htm * Vector4(kSectionRadius * cos(angle), kSectionRadius * sin(angle), 0, 1);
			points[i].PointPos.x = point.x;
			points[i].PointPos.y = point.y;
			points[i].PointPos.z = point.z;
			// Texture coordinates.
			points[i].Properties.TextureCoordinates.x = static_cast<DWORD>((((a % kNumSectionPoints) * texture->SLIXSize * kTextureFactorX) / kNumSectionPoints)) % texture->SLIXSize;
			points[i].Properties.TextureCoordinates.y = (distance_over_curve * texture->SLIYSize) / kTextureHeightMeters; //pose.position() static_cast<DWORD>((((j / kNumSectionPoints) * texture->SLIYSize * kTextureFactorY) / kNumSections)) % texture->SLIYSize;
			++i;
		}
		last_section_center = pose.position();
	}

	// Initialize point properties.
	for (int j = 0; j < num_points; ++j) {
		points[j].Properties.Light1 = 0;
		points[j].Properties.Light2 = 0;
		points[j].PointFlags = 0;
		points[j].OrigPointPos = points[j].PointPos;
		points[j].Normal.x = 1.0f / sqrt(3);
		points[j].Normal.y = 1.0f / sqrt(3);
		points[j].Normal.z = -1.0f / sqrt(3);
		points[j].OrigNormal = points[j].Normal;		
	}

	// Create polygons.
	const int num_polys = kNumSectionPoints * 2 * (kNumSections - 1);
	RENDERPOLY *polys = new RENDERPOLY[num_polys];
	for (int k = 0; k < num_polys; ++k) {
		polys[k].N = 3;
		
		// Render flat colors
		// polys[k].Type = 1; // 1: flat
		// polys[k].PolyFlags = kBackFaceCulled;
		// DWORD b = ((k / 2) % 2 + 1) > 0 ? 0xff : 0;
		// DWORD g = ((k / 4) % 2) > 0 ? 0xff00 : 0;
		// DWORD r = ((k / 8) % 2) > 0 ? 0xff0000 : 0;
		// *(DWORD *)&polys[k].Texture1 = b | g | r;
		
		// Render textures
		polys[k].Type = 3; // | kSptPerspectiveCorrected;
		polys[k].Texture1 = texture;
		polys[k].PolyFlags = kBackFaceCulled | kVertex1NormalFromPoint | kVertex2NormalFromPoint | kVertex3NormalFromPoint;
		
		// Assign points to polygon.
		const int cur_section_index = k / (2 * kNumSectionPoints);
		const int cur_section_point_index = k % kNumSectionPoints;
		if (k % (2 * kNumSectionPoints) >= kNumSectionPoints) {
			// Second section: two points in this section, one in the previous section (anticlockwise for normal to point outside).
			polys[k].P1 = &points[(cur_section_index + 1) * kNumSectionPoints + ((cur_section_point_index + 1) % kNumSectionPoints)];
			polys[k].P3 = &points[(cur_section_index + 1) * kNumSectionPoints + cur_section_point_index];
			polys[k].P2 = &points[cur_section_index * kNumSectionPoints + cur_section_point_index];
		} else {
			// First section: two points in this section, one in the next section (anticlockwise for normal to point outside).
			polys[k].P1 = &points[cur_section_index * kNumSectionPoints + cur_section_point_index];
			polys[k].P3 = &points[cur_section_index * kNumSectionPoints + ((cur_section_point_index + 1) % kNumSectionPoints)];
			polys[k].P2 = &points[(cur_section_index + 1) * kNumSectionPoints + ((cur_section_point_index + 1) % kNumSectionPoints)];
		}
	}
	
	// Polygon normals.
	for (int l = 0; l < num_polys; ++l) {
		CalcPolyNormal(polys[l]);
	}
	
	// Point normals from polygon normals.
	for (int m = 0; m < num_points; ++m) {
		points[m].Normal.x = 0;
		points[m].Normal.y = 0;
		points[m].Normal.z = 0;
	}
	for (l = 0; l < num_polys; ++l) {
		polys[l].P1->Normal.x += polys[l].PNormal.x;
		polys[l].P1->Normal.y += polys[l].PNormal.y;
		polys[l].P1->Normal.z += polys[l].PNormal.z;
		polys[l].P2->Normal.x += polys[l].PNormal.x;
		polys[l].P2->Normal.y += polys[l].PNormal.y;
		polys[l].P2->Normal.z += polys[l].PNormal.z;
		polys[l].P3->Normal.x += polys[l].PNormal.x;
		polys[l].P3->Normal.y += polys[l].PNormal.y;
		polys[l].P3->Normal.z += polys[l].PNormal.z;
	}
	for (m = 0; m < num_points; ++m) {
		float norm = sqrt(points[m].Normal.x * points[m].Normal.x + points[m].Normal.y * points[m].Normal.y + points[m].Normal.z * points[m].Normal.z);
		points[m].Normal.x /= norm;
		points[m].Normal.y /= norm;
		points[m].Normal.z /= norm;
		points[m].OrigNormal = points[m].Normal;
	}
	
	RENDERMESH mesh;
	mesh.polygons = num_polys;
	mesh.polydata = polys;
	mesh.points = num_points;
	mesh.pointdata = points;
	mesh.Pivot.x = 0.1;
	mesh.Pivot.y = 0;
	mesh.Pivot.z = 1;
	mesh.MeshName[0] = '1';
	mesh.MeshName[1] = '\0';
	
	RENDERCAMERA camera;
	camera.CamPos.x = 0; //2;
	camera.CamPos.y = 0.2; //1.1;
	camera.CamPos.z = -2; //-2;
	camera.CamOrientation.x = 0; //-3.14159 / 4;
	camera.CamOrientation.y = -3.14159 / 6; //-3.14159 / 8;
	camera.CamOrientation.z = 0; //0;
	camera.CamFocus = 200;
	camera.sin_alpha = sin(camera.CamOrientation.x);
	camera.cos_alpha = cos(camera.CamOrientation.x);
	camera.sin_beta = sin(camera.CamOrientation.y);
	camera.cos_beta = cos(camera.CamOrientation.y);
	camera.sin_gamma = sin(camera.CamOrientation.z);
	camera.cos_gamma = cos(camera.CamOrientation.z);
	camera.CamName[0] = 'c';
	camera.CamName[1] = '\0';
	
	RENDERLIGHT light;
	light.LightIntensity = 126;
	light.LightName[0] = 'l';
	light.LightName[1] = '\0';
	SetLightPos(0, 0, 0, &light);
	SetLightTarget(-0.1, 0, -0.1, &light);
	
	RENDERWORLD world;
	world.NumMeshes = 1;
	world.MeshArray = &mesh;
	world.NumCameras = 1;
	world.CameraArray = &camera;
	world.NumLights = 1;
	world.LightArray = &light;
	world.WorldDestructor = NULL;

	SetActiveCamera(&camera);
	
	SLI *back_buffer = CreateSLI(kDisplayWidth, kDisplayHeight, kDisplayBpp, 1);
	error = CreateVideoSLI(kDisplayWidth, kDisplayHeight, kDisplayBpp);
	if (error) {
		goto error_exit;
	}
	
	float light_x = 0, light_y = 0, light_z = 0;	
	float light_target_x = 0, light_target_y = 0, light_target_z = 0;	
	float t = 0.0f, t_inc = 0.01f;
	StartFrameRate();
	while(loop) {		
		ClearCurrentFrame(back_buffer, 0);
		RenderUniverse(&world, back_buffer);
		Blit(GetVideoSLI(), back_buffer);
		ShowPage();
		RotateMesh(0, t, &mesh);
		IncFloat(&t, &t_inc);
		GetElapsedTime();
		IncFrameCounter();
	}
	StopFrameRate();

error_exit:
	DestroyVideoSLI(3);
	if (error) {
		printf("Error code: 0x%dhn\n", error);
	}
	ShowFrameRateResult();
	return error;
}