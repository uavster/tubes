#include "tdl.h"
#include "tinymath.h"
#include "vector3.hpp"
#include "pose3.hpp"
#include "htmat4.hpp"
#include "interp.hpp"
#include "palette.hpp"

#define kDisplayWidth	320
#define kDisplayHeight	200
#define kDisplayBpp		32

#define kSectionRadius	0.2f

#define kGlowingBandSpeed	100	// pixels/second

#define kFxFirstIndex 	(255 - 15)
#define kRampLength		4
#define kFxColor		0xffffe080

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

int bright_bar_y = -1;

void AnimateTexture(SLI *texture, SLI *original_texture, float new_time) {	
	const int kBrightBarHeight = 10;
	
	const BYTE *max_dst = static_cast<BYTE *>(GetCurrentFramePtr(texture)) + texture->SLIYSize * texture->SLIPitch;
	if (bright_bar_y >= 0) {
		// Restore original colors.
		BYTE *src = static_cast<BYTE *>(GetCurrentFramePtr(original_texture)) + bright_bar_y * original_texture->SLIPitch;
		BYTE *dst = static_cast<BYTE *>(GetCurrentFramePtr(texture)) + bright_bar_y * texture->SLIPitch;
		for (int y = 0; y < kBrightBarHeight; ++y) {
			for (int x = 0; x < texture->SLIXSize; ++x, ++src, ++dst) {
				*dst = *src;
			}
			dst += texture->SLIPitch - texture->SLIXSize;
			src += original_texture->SLIPitch - original_texture->SLIXSize;
			if (dst >= max_dst) {
				dst = static_cast<BYTE *>(GetCurrentFramePtr(texture));
				src = static_cast<BYTE *>(GetCurrentFramePtr(original_texture));
			}
		}
	}
	// Generate bright band.
	bright_bar_y = static_cast<int>(new_time * kGlowingBandSpeed) % texture->SLIYSize;
	BYTE *dst = static_cast<BYTE *>(GetCurrentFramePtr(texture)) + bright_bar_y * texture->SLIPitch;
	for (int y = 0; y < kBrightBarHeight; ++y) {
		BYTE color = y + (255 - kBrightBarHeight);
		for (int x = 0; x < texture->SLIXSize; ++x, ++dst) {
			*dst = color;
		}
		dst += texture->SLIPitch - texture->SLIXSize;
		if (dst >= max_dst) {
			dst = static_cast<BYTE *>(GetCurrentFramePtr(texture));
		}
	}
}

void MakeLightsGlow(SLI *sli, Pose3Interpolator &pose_interpolator, RENDERCAMERA &camera, float time, Interpolator<float> &t_from_text_v, const SLI *texture_sli, int num_bands, DWORD *band_palette) {
	DWORD *sli_data = static_cast<DWORD *>(GetCurrentFramePtr(sli));
	int glow_sli_pos = 0;
	SLR rect[16]; // Max bands
	SLI *glow_sli[16]; // Max bands
	for (int i = 0; i < num_bands; ++i) {
		Pose3 pose = pose_interpolator.getValue(t_from_text_v.getValue(time * kGlowingBandSpeed + texture_sli->SLIYSize * i)); //(5.5 / 5.8));
		RENDERPOINT point;
		point.PointPos.x = pose.position().x;
		point.PointPos.y = pose.position().y;
		point.PointPos.z = pose.position().z;
		POINT2D projection;
		ProjectPointOnCamera(&point, &camera, &projection);
		int py = static_cast<int>(projection.y);
		int px = static_cast<int>(projection.x);
		float point_cam_dx = camera.CamPos.x - point.PointPos.x;
		float point_cam_dy = camera.CamPos.y - point.PointPos.y;
		float point_cam_dz = camera.CamPos.z - point.PointPos.z;
		float point_cam_distance = point_cam_dx * point_cam_dx + point_cam_dy * point_cam_dy + point_cam_dz * point_cam_dz;
		const int side = camera.CamFocus * 1.5f / (point_cam_distance + kSectionRadius * 0.5f) + 5;
		
		int first_y = (py - side > 0) ? py - side : 0;
		int last_y = (py + side < sli->SLIYSize) ? py + side : sli->SLIYSize - 1;
		int first_x = (px - side > 0) ? px - side : 0;
		int last_x = (px + side < sli->SLIXSize) ? px + side : sli->SLIXSize - 1;
		glow_sli[i] = CreateSLI(last_x - first_x + 1, last_y - first_y + 1, 8, 1);		
		if (glow_sli[i] == NULL) {
			return;
		}
		for (int y = first_y; y < last_y; ++y) {
			BYTE *glow_data_ptr0 = static_cast<BYTE *>(GetCurrentFramePtr(glow_sli[i])) + glow_sli[i]->SLIPitch * (y - first_y);
			DWORD *p0 = &sli_data[sli->SLIPitch * y];
			const SPAN *span = GetFirstSpanInLine(sli, y);
			while((DWORD)span != -1) {
				if (span->SPX2 > first_x && span->SPX1 < last_x) {
					BYTE *glow_data_ptr = glow_data_ptr0 + (span->SPX1 - first_x);
					DWORD *p = p0 + (span->SPX1 > first_x ? span->SPX1 : first_x);
					DWORD *p1 = p0 + (span->SPX2 < last_x ? span->SPX2 : last_x);
					if (*p >= kFxColor) {
						if (*p1 >= kFxColor) {
							while(p <= p1) { *(glow_data_ptr++) = (int)*(p++) >> 16; }
						} else {
							while(p <= p1 && *p >= kFxColor) { *(glow_data_ptr++) = (int)*(p++) >> 16; }
						}
					} else {
						if (*p1 >= kFxColor) {
							while(p <= p1 && *p < kFxColor) { ++glow_data_ptr; ++p; }
							while(p <= p1) { *(glow_data_ptr++) = (int)*(p++) >> 16; }
						}
					}
				}
				span = (const SPAN *)span->SPNext;
			}
		}
		SetPos(glow_sli[i], glow_sli_pos, 0);
		glow_sli_pos += 2 * side + 1;

		rect[i].SLRR1.SLPX = first_x;
		rect[i].SLRR1.SLPY = first_y;
		rect[i].SLRR2.SLPX = last_x;
		rect[i].SLRR2.SLPY = last_y;
	}
	SLI *glow_sli_smoothed[16];
	for (int h = 0; h < num_bands; ++h) {
		glow_sli_smoothed[h] = CreateSLI(glow_sli[h]->SLIXSize, glow_sli[h]->SLIYSize, 8, 1);
		SetPos(glow_sli_smoothed[h], rect[h].SLRR1.SLPX, rect[h].SLRR1.SLPY); //glow_sli[h]->SLIPos.SLRR1.SLPX, glow_sli[h]->SLIPos.SLRR1.SLPY);
		SetPalette(glow_sli_smoothed[h], (BYTE *)band_palette);
		Soften8((BYTE *)GetCurrentFramePtr(glow_sli[h]) + glow_sli[h]->SLIXSize * 2, glow_sli[h]->SLIXSize, glow_sli[h]->SLIYSize - 4, (BYTE *)GetCurrentFramePtr(glow_sli_smoothed[h]) + glow_sli_smoothed[h]->SLIXSize * 2);
		Soften8((BYTE *)GetCurrentFramePtr(glow_sli_smoothed[h]) + glow_sli_smoothed[h]->SLIXSize * 2, glow_sli_smoothed[h]->SLIXSize, glow_sli_smoothed[h]->SLIYSize - 4, (BYTE *)GetCurrentFramePtr(glow_sli[h]) + glow_sli[h]->SLIXSize * 2);
		Soften8((BYTE *)GetCurrentFramePtr(glow_sli[h]) + glow_sli[h]->SLIXSize * 2, glow_sli[h]->SLIXSize, glow_sli[h]->SLIYSize - 4, (BYTE *)GetCurrentFramePtr(glow_sli_smoothed[h]) + glow_sli_smoothed[h]->SLIXSize * 2);
		Soften8((BYTE *)GetCurrentFramePtr(glow_sli_smoothed[h]) + glow_sli_smoothed[h]->SLIXSize * 2, glow_sli_smoothed[h]->SLIXSize, glow_sli_smoothed[h]->SLIYSize - 4, (BYTE *)GetCurrentFramePtr(glow_sli[h]) + glow_sli[h]->SLIXSize * 2);
		Soften8((BYTE *)GetCurrentFramePtr(glow_sli[h]) + glow_sli[h]->SLIXSize * 2, glow_sli[h]->SLIXSize, glow_sli[h]->SLIYSize - 4, (BYTE *)GetCurrentFramePtr(glow_sli_smoothed[h]) + glow_sli_smoothed[h]->SLIXSize * 2);
		DestroySLI(glow_sli[h]);
	}
	static DWORD band_rect_colors[] = { 0xffff0000, 0xff00ff00, 0xff0000ff };
	for (int j = 0; j < num_bands; ++j) {
		// DrawLine(sli, rect[j].SLRR1.SLPX, rect[j].SLRR1.SLPY, rect[j].SLRR2.SLPX, rect[j].SLRR1.SLPY, band_rect_colors[j]);
		// DrawLine(sli, rect[j].SLRR1.SLPX, rect[j].SLRR1.SLPY, rect[j].SLRR1.SLPX, rect[j].SLRR2.SLPY, band_rect_colors[j]);
		// DrawLine(sli, rect[j].SLRR1.SLPX, rect[j].SLRR2.SLPY, rect[j].SLRR2.SLPX, rect[j].SLRR2.SLPY, band_rect_colors[j]);
		// DrawLine(sli, rect[j].SLRR2.SLPX, rect[j].SLRR1.SLPY, rect[j].SLRR2.SLPX, rect[j].SLRR2.SLPY, band_rect_colors[j]);		
		SetBlitType(glow_sli_smoothed[j], SLIBlitAddSaturate);
		Blit(sli, glow_sli_smoothed[j]);
		// DrawLine(sli, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPY, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPY, band_rect_colors[j]);
		// DrawLine(sli, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPY, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPY, band_rect_colors[j]);
		// DrawLine(sli, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPY, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPY, band_rect_colors[j]);
		// DrawLine(sli, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR1.SLPY, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPX, glow_sli_smoothed[j]->SLIPos.SLRR2.SLPY, band_rect_colors[j]);		
		DestroySLI(glow_sli_smoothed[j]);
	}
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
	const float kLastToFirstKeypointTime = 0.25f;
	pose_trajectory.setPeriodic(kLastToFirstKeypointTime);

	Pose3CubicCatmullRomInterpolator pose_interpolator;
	pose_interpolator.setTrajectory(pose_trajectory);
	
	float trajectory_start_t = pose_trajectory.cbegin()->time();
	float trajectory_end_t = (--pose_trajectory.cend())->time() + kLastToFirstKeypointTime;
	float trajectory_length = trajectory_end_t - trajectory_start_t;
	const int kNumSections = 80;
	const int kNumSectionPoints = 5;
	const float kTextureFactorX = 0.25;
	// const float kTextureFactorY = kNumSections / 5.0f;
	const float kTextureHeightMeters = 5.8;

/*	
	SLI *band_slis[32];	// Max bands
	int num_bright_bands = 0;
	void *band_frame_data = NULL;
*/
	
	if (kNumSections < 2) {
		printf("kNumSections must be >= 3.");
		goto error_exit;
	}

	SLI *original_texture = NULL;
	SLI *texture = NULL;
	SLI *back_buffer = NULL;
	
	texture = LoadGFX("texture.pcx", 8);
	if (texture == NULL) {
		error = -1;
		goto error_exit;
	}
	SortSLIPaletteByLuminance(texture);

	// Reserve 16 colors for on-texture lighting effects.	
	CompressSLIPalette(texture, 0, 255, 0, kFxFirstIndex - 1);
	// Fill in the fx range with a constant color with alpha ramps.
	vector<DWORD> fx_palette(256, 0);
	CreateColorRamp(fx_palette.data(), kFxFirstIndex + kRampLength + 1, kFxColor, 255 - kRampLength - 1, kFxColor);
	CreateColorRamp(fx_palette.data(), kFxFirstIndex, kFxColor, kFxFirstIndex + kRampLength, kFxColor);
	CreateColorRamp(fx_palette.data(), 255 - kRampLength, kFxColor, 255, kFxColor);
	
	// Mix fx and texture palettes.
	MixWithSLIPalette(texture, fx_palette.data());
	
	const int kLightingLevels = 129;
	CreateLitTableSLI(texture, BLACK2WHITE, kLightingLevels);
	
	// Set all lighting levels to max in table to remove shadows.
	for (int p = 0; p < kLightingLevels; ++p) {
		for (int q = kFxFirstIndex; q <= 255; ++q) {
			((DWORD *)(texture->SLILitTable))[(p << 8) + q] = kFxColor;
		}
	}

	original_texture = CreateSLI(texture->SLIXSize, texture->SLIYSize, texture->SLIColorBits << 3, GetNumFrames(texture));
	if (original_texture == NULL) {
		error = -2;
		goto error_exit;
	}
	Blit(original_texture, texture);
	
	TimeSignal<float> t_from_text_y_signal;
	t_from_text_y_signal.setPeriodic(/*last_to_first_keypoint_time=*/0);
	// Cannot be a periodic cubic interpolator because there is a step between 
	// end and start that produces ripple in the interpolated values around it.
	// The ripple results in negative values and the location of each band
	// dashes suddenly back when passing over the tube's origin.
	LinearInterpolator<float> t_from_text_y(t_from_text_y_signal);
	
	const int num_points = kNumSections * kNumSectionPoints;
	RENDERPOINT *points = new RENDERPOINT[num_points];
	int i = 0;
	Vector3 last_section_center = pose_interpolator.getValue(0).position();
	float distance_over_curve = 0;
	for (int n = 0; n < kNumSections; ++n)  {
		float t = trajectory_start_t + (trajectory_length * n) / (kNumSections - 1);
		Pose3 pose = pose_interpolator.getValue(t);
		HTMatrix4 htm(pose.orientation().getRotMatrix(), pose.position());
		distance_over_curve += (pose.position() - last_section_center).modulus();
		float texture_y = (distance_over_curve * texture->SLIYSize) / kTextureHeightMeters;
		t_from_text_y_signal.addKeyPoint(texture_y, t);
		// Sample the shape to create points around the current pose.
		for (int a = 0; a < kNumSectionPoints; ++a) {
			float angle = -M_PI + (a * 2 * M_PI) / kNumSectionPoints;
			Vector4 point = htm * Vector4(kSectionRadius * cos(angle), kSectionRadius * sin(angle), 0, 1);
			points[i].PointPos.x = point.x;
			points[i].PointPos.y = point.y;
			points[i].PointPos.z = point.z;
			// Texture coordinates.
			points[i].Properties.TextureCoordinates.x = static_cast<DWORD>((((a % kNumSectionPoints) * texture->SLIXSize * kTextureFactorX) / kNumSectionPoints)) % texture->SLIXSize;
			points[i].Properties.TextureCoordinates.y = texture_y; //pose.position() static_cast<DWORD>((((j / kNumSectionPoints) * texture->SLIYSize * kTextureFactorY) / kNumSections)) % texture->SLIYSize;
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
	mesh.Pivot.x = 0;
	mesh.Pivot.y = 1;
	mesh.Pivot.z = 0;
	mesh.MeshName[0] = '1';
	mesh.MeshName[1] = '\0';
	
	RENDERCAMERA camera;
	camera.CamFocus = 200;
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

	const int num_bright_bands = static_cast<int>(distance_over_curve / kTextureHeightMeters);
/*	
	// Create SLIs to hold band masks.
	band_frame_data = malloc(num_bright_bands * (kDisplayWidth * kDisplayHeight));
	if (band_frame_data == NULL) {
		error = -4;
		goto error_exit;
	}
	for (int bi = 0; bi < num_bright_bands; ++bi) {
		band_slis[bi] = CreateVoidSLI(kDisplayWidth, kDisplayHeight, 8, 1);
		if (band_slis[bi] == NULL) {
			error = -5;
			goto error_exit;
		}
		SetFrameData(band_slis[bi], static_cast<BYTE *>(band_frame_data) + bi * (kDisplayWidth * kDisplayHeight));
	}
*/
	
	back_buffer = CreateSLI(kDisplayWidth, kDisplayHeight, kDisplayBpp, 1);
	if (back_buffer == NULL) {
		error = -3;
		goto error_exit;
	}
	error = CreateVideoSLI(kDisplayWidth, kDisplayHeight, kDisplayBpp);
	if (error) {
		goto error_exit;
	}
	
	DWORD bright_band_palette[256];
	CreateColorRamp(bright_band_palette, 0, 0xff000000, 255, 0xffffffff);
	
	float light_x = 0, light_y = 0, light_z = 0;	
	float light_target_x = 0, light_target_y = 0, light_target_z = 0;	
	float t = 0.0f, t_inc = 0.01f;
	StartFrameRate();
	while(loop) {
		const float kCamPosRadius = 4;
		SetCameraPos(mesh.Pivot.x + kCamPosRadius * cos(t), mesh.Pivot.y, mesh.Pivot.z + kCamPosRadius * sin(t), &camera);
		SetCameraTarget(mesh.Pivot.x, mesh.Pivot.y - 4, mesh.Pivot.z, &camera);
		SetLightPos(mesh.Pivot.x + kCamPosRadius * cos(t + M_PI), mesh.Pivot.y, mesh.Pivot.z + kCamPosRadius * sin(t + M_PI), &light);
		SetLightTarget(0, 0, 0, &light);
		ClearCurrentFrame(back_buffer, 0);
		SetRenderMethode(RENDER_SOLID);
		AnimateTexture(texture, original_texture, t);
		RenderUniverse(&world, back_buffer);
		MakeLightsGlow(back_buffer, pose_interpolator, camera, t, t_from_text_y, texture, num_bright_bands, bright_band_palette);
		Blit(GetVideoSLI(), back_buffer);
		ShowPage();
		GetElapsedTime();
		IncFloat(&t, &t_inc);
		IncFrameCounter();
	}
	StopFrameRate();

error_exit:
	DestroyVideoSLI(3);	
	if (error) {
		printf("Error code: 0x%dhn\n", error);
	}
/*
	if (band_frame_data != NULL) {
		free(band_frame_data);
	} 
	{
		for (int i = 0; i < num_bright_bands; ++i) {
			DestroySLI(band_slis[i]);
		}
	}
*/
	if (back_buffer != NULL) {
		DestroySLI(back_buffer);
	}
	if (texture != NULL) {
		DestroySLI(texture);
	}
	if (original_texture != NULL) {
		DestroySLI(original_texture);
	}
	ShowFrameRateResult();
	return error;
}