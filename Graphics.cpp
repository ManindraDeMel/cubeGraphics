#pragma once
#include "algorithm"
#include "Binary.h" // Made by Manindra de Mel // begun 2019, somewhat completed 2020
#include <vector>
using namespace std;

struct vectors3d {
	float x, y, z;
};

struct triangle {
	vectors3d p[3];
	wchar_t sym;
	short col;
};

struct mesh {
	vector<triangle> tris;
};
struct matrix4x4
{
	float arr[4][4] = { 0 };
};

class Engine : public GameEngine
{
private:
	int limit = 0;
	mesh meshCube;
	matrix4x4 matProjection;
	vectors3d vCamera;
	vectors3d vLookDirection;
	float fTheta = 0;
	bool manualControl = false;
	void MatrixMultplication(vectors3d& i, vectors3d& o, matrix4x4& m)
	{
		o.x = i.x * m.arr[0][0] + i.y * m.arr[1][0] + i.z * m.arr[2][0] + m.arr[3][0];
		o.y = i.x * m.arr[0][1] + i.y * m.arr[1][1] + i.z * m.arr[2][1] + m.arr[3][1];
		o.z = i.x * m.arr[0][2] + i.y * m.arr[1][2] + i.z * m.arr[2][2] + m.arr[3][2];
		float w = i.x * m.arr[0][3] + i.y * m.arr[1][3] + i.z * m.arr[2][3] + m.arr[3][3];

		if (w != 0.0f)
		{
			o.x /= w; o.y /= w; o.z /= w;
		}
	}

	matrix4x4 Matrix_MakeIdentity()
	{
		matrix4x4 matrix;
		matrix.arr[0][0] = 1.0f;
		matrix.arr[1][1] = 1.0f;
		matrix.arr[2][2] = 1.0f;
		matrix.arr[3][3] = 1.0f;
		return matrix;
	}

	matrix4x4 Matrix_MakeRotationX(float fAngleRad)
	{
		matrix4x4 matrix;
		matrix.arr[0][0] = 1.0f;
		matrix.arr[1][1] = cosf(fAngleRad);
		matrix.arr[1][2] = sinf(fAngleRad);
		matrix.arr[2][1] = -sinf(fAngleRad);
		matrix.arr[2][2] = cosf(fAngleRad);
		matrix.arr[3][3] = 1.0f;
		return matrix;
	}

	matrix4x4 Matrix_MakeRotationY(float fAngleRad)
	{
		matrix4x4 matrix;
		matrix.arr[0][0] = cosf(fAngleRad);
		matrix.arr[0][2] = sinf(fAngleRad);
		matrix.arr[2][0] = -sinf(fAngleRad);
		matrix.arr[1][1] = 1.0f;
		matrix.arr[2][2] = cosf(fAngleRad);
		matrix.arr[3][3] = 1.0f;
		return matrix;
	}

	matrix4x4 Matrix_MakeRotationZ(float fAngleRad)
	{
		matrix4x4 matrix;
		matrix.arr[0][0] = cosf(fAngleRad);
		matrix.arr[0][1] = sinf(fAngleRad);
		matrix.arr[1][0] = -sinf(fAngleRad);
		matrix.arr[1][1] = cosf(fAngleRad);
		matrix.arr[2][2] = 1.0f;
		matrix.arr[3][3] = 1.0f;
		return matrix;
	}

	matrix4x4 Matrix_MakeTranslation(float x, float y, float z)
	{
		matrix4x4 matrix;
		matrix.arr[0][0] = 1.0f;
		matrix.arr[1][1] = 1.0f;
		matrix.arr[2][2] = 1.0f;
		matrix.arr[3][3] = 1.0f;
		matrix.arr[3][0] = x;
		matrix.arr[3][1] = y;
		matrix.arr[3][2] = z;
		return matrix;
	}

	matrix4x4 Matrix_MakeProjection(float fFovDegrees, float fAspectRatio, float fNear, float fFar)
	{
		float fFovRad = 1.0f / tanf(fFovDegrees * 0.5f / 180.0f * 3.14159f);
		matrix4x4 matrix;
		matrix.arr[0][0] = fAspectRatio * fFovRad;
		matrix.arr[1][1] = fFovRad;
		matrix.arr[2][2] = fFar / (fFar - fNear);
		matrix.arr[3][2] = (-fFar * fNear) / (fFar - fNear);
		matrix.arr[2][3] = 1.0f;
		matrix.arr[3][3] = 0.0f;
		return matrix;
	}

	matrix4x4 Matrix_MultiplyMatrix(matrix4x4& m1, matrix4x4& m2)
	{
		matrix4x4 matrix;
		for (int c = 0; c < 4; c++)
			for (int r = 0; r < 4; r++)
				matrix.arr[r][c] = m1.arr[r][0] * m2.arr[0][c] + m1.arr[r][1] * m2.arr[1][c] + m1.arr[r][2] * m2.arr[2][c] + m1.arr[r][3] * m2.arr[3][c];
		return matrix;
	}

	matrix4x4 Matrix_PointAt(vectors3d& pos, vectors3d& target, vectors3d& up)
	{
		// Calculate new forward direction
		vectors3d newForward = Vector_Sub(target, pos);
		newForward = Vector_Normalise(newForward);

		// Calculate new Up direction
		vectors3d a = Vector_Mul(newForward, Vector_DotProduct(up, newForward));
		vectors3d newUp = Vector_Sub(up, a);
		newUp = Vector_Normalise(newUp);

		// New Right direction is easy, its just cross product
		vectors3d newRight = Vector_CrossProduct(newUp, newForward);

		// Construct Dimensioning and Translation Matrix	
		matrix4x4 matrix;
		matrix.arr[0][0] = newRight.x;	matrix.arr[0][1] = newRight.y;	matrix.arr[0][2] = newRight.z;	matrix.arr[0][3] = 0.0f;
		matrix.arr[1][0] = newUp.x;		matrix.arr[1][1] = newUp.y;		matrix.arr[1][2] = newUp.z;		matrix.arr[1][3] = 0.0f;
		matrix.arr[2][0] = newForward.x;	matrix.arr[2][1] = newForward.y;	matrix.arr[2][2] = newForward.z;	matrix.arr[2][3] = 0.0f;
		matrix.arr[3][0] = pos.x;			matrix.arr[3][1] = pos.y;			matrix.arr[3][2] = pos.z;			matrix.arr[3][3] = 1.0f;
		return matrix;

	}

	matrix4x4 Matrix_QuickInverse(matrix4x4& m) // Only for Rotation/Translation Matrices
	{
		matrix4x4 matrix;
		matrix.arr[0][0] = m.arr[0][0]; matrix.arr[0][1] = m.arr[1][0]; matrix.arr[0][2] = m.arr[2][0]; matrix.arr[0][3] = 0.0f;
		matrix.arr[1][0] = m.arr[0][1]; matrix.arr[1][1] = m.arr[1][1]; matrix.arr[1][2] = m.arr[2][1]; matrix.arr[1][3] = 0.0f;
		matrix.arr[2][0] = m.arr[0][2]; matrix.arr[2][1] = m.arr[1][2]; matrix.arr[2][2] = m.arr[2][2]; matrix.arr[2][3] = 0.0f;
		matrix.arr[3][0] = -(m.arr[3][0] * matrix.arr[0][0] + m.arr[3][1] * matrix.arr[1][0] + m.arr[3][2] * matrix.arr[2][0]);
		matrix.arr[3][1] = -(m.arr[3][0] * matrix.arr[0][1] + m.arr[3][1] * matrix.arr[1][1] + m.arr[3][2] * matrix.arr[2][1]);
		matrix.arr[3][2] = -(m.arr[3][0] * matrix.arr[0][2] + m.arr[3][1] * matrix.arr[1][2] + m.arr[3][2] * matrix.arr[2][2]);
		matrix.arr[3][3] = 1.0f;
		return matrix;
	}

	vectors3d Vector_Add(vectors3d& v1, vectors3d& v2)
	{
		return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	}

	vectors3d Vector_Sub(vectors3d& v1, vectors3d& v2)
	{
		return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
	}

	vectors3d Vector_Mul(vectors3d& v1, float k)
	{
		return { v1.x * k, v1.y * k, v1.z * k };
	}

	vectors3d Vector_Div(vectors3d& v1, float k)
	{
		return { v1.x / k, v1.y / k, v1.z / k };
	}

	float Vector_DotProduct(vectors3d& v1, vectors3d& v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}

	float Vector_Length(vectors3d& v)
	{
		return sqrtf(Vector_DotProduct(v, v));
	}

	vectors3d Vector_Normalise(vectors3d& v)
	{
		float l = Vector_Length(v);
		return { v.x / l, v.y / l, v.z / l };
	}

	vectors3d Vector_CrossProduct(vectors3d& v1, vectors3d& v2)
	{
		vectors3d v;
		v.x = v1.y * v2.z - v1.z * v2.y;
		v.y = v1.z * v2.x - v1.x * v2.z;
		v.z = v1.x * v2.y - v1.y * v2.x;
		return v;
	}
	CHAR_INFO GetColour(float lum)
	{
		short bg_col, fg_col;
		wchar_t sym;
		int pixel_bw = (int)(13.0f * lum);
		switch (pixel_bw)
		{
		case 0: bg_col = BG_BLACK; fg_col = FG_BLACK; sym = PIXEL_SOLID; break;

		case 1: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_QUARTER; break;
		case 2: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_HALF; break;
		case 3: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_THREEQUARTERS; break;
		case 4: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_SOLID; break;

		case 5: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_QUARTER; break;
		case 6: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_HALF; break;
		case 7: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_THREEQUARTERS; break;
		case 8: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_SOLID; break;

		case 9:  bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_QUARTER; break;
		case 10: bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_HALF; break;
		case 11: bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_THREEQUARTERS; break;
		case 12: bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_SOLID; break;
		default:
			bg_col = BG_BLACK; fg_col = FG_BLACK; sym = PIXEL_SOLID;
		}

		CHAR_INFO c;
		c.Attributes = bg_col | fg_col;
		c.Char.UnicodeChar = sym;
		return c;
	}

	mesh createCubePoints(float bottomx, float bottomy, float bottomz) {
		mesh MeshCube;
		MeshCube.tris = {
			// SOUTH
			{ bottomx, bottomy, bottomz,    bottomx, bottomy + 1, bottomz,    bottomx + 1, bottomy + 1, bottomz },
			{ bottomx, bottomy, bottomz,    bottomx + 1, bottomy + 1, bottomz,    bottomx + 1, bottomy, bottomz },

			// EAST                                                      
			{ bottomx + 1, bottomy, bottomz,    bottomx + 1, bottomy + 1, bottomz,    bottomx + 1, bottomy + 1, bottomz + 1},
			{bottomx + 1, bottomy,bottomz,     bottomx + 1, bottomy + 1, bottomz + 1,    bottomx + 1, bottomy, bottomz + 1},

			// NORTH                                                     
			{ bottomx + 1, bottomy, bottomz + 1,    bottomx + 1, bottomy + 1, bottomz + 1,    bottomx, bottomy + 1, bottomz + 1 },
			{ bottomx + 1, bottomy, bottomz + 1,    bottomx, bottomy + 1, bottomz + 1,    bottomx, bottomy, bottomz + 1 },

			// WEST                                                      
			{ bottomx, bottomy, bottomz + 1,    bottomx, bottomy + 1, bottomz + 1,    bottomx, bottomy + 1, bottomz },
			{ bottomx, bottomy, bottomz + 1,    bottomx, bottomy + 1, bottomz,    bottomx, bottomy, bottomz },

			// TOP                                                       
			{ bottomx, bottomy + 1, bottomz,    bottomx, bottomy + 1, bottomz + 1,    bottomx + 1, bottomy + 1, bottomz + 1},
			{ bottomx, bottomy + 1, bottomz,    bottomx + 1, bottomy + 1, bottomz + 1,    bottomx + 1, bottomy + 1, bottomz },

			// BOTTOM                                                    
			{ bottomx + 1, bottomy, bottomz + 1,   bottomx, bottomy, bottomz + 1,    bottomx, bottomy, bottomz },
			{ bottomx + 1, bottomy, bottomz + 1,    bottomx, bottomy, bottomz,    bottomx + 1, bottomy, bottomz }
		};
		return MeshCube;
	}
	void createCubeMesh(float point1, float point2, float point3) {
		for (int i = 0; i < 12; i++) {
			meshCube.tris.push_back(createCubePoints(point1, point2, point3).tris[i]);
		}
	}

public:
	Engine() {
		m_sAppName = L"3D Engine";
	}
	bool OnUserUpdate(float fElapsedTime) override {		
		

		Fill(0, 0, ScreenWidth(), ScreenHeight(), PIXEL_SOLID, FG_BLACK);
		matrix4x4 rotateZ, rotateX;
		if (manualControl) {
			if (GetKeyState('A') & 0x8000)
			{
				fTheta -= 1 * fElapsedTime;
			}
			else if (GetKeyState('D') & 0x8000) {
				fTheta += 1 * fElapsedTime;
			}
		}
		/////////////////////////////////
		else if (!manualControl) {
			fTheta += 1 * fElapsedTime; 
		}
		/////////////////////////////////
		// Rotation Z
		rotateZ.arr[0][0] = cosf(fTheta);
		rotateZ.arr[0][1] = sinf(fTheta);
		rotateZ.arr[1][0] = -sinf(fTheta);
		rotateZ.arr[1][1] = cosf(fTheta);
		rotateZ.arr[2][2] = 1;
		rotateZ.arr[3][3] = 1;

		// Rotation X
		rotateX.arr[0][0] = 1;
		rotateX.arr[1][1] = cosf(fTheta * 0.5f);
		rotateX.arr[1][2] = sinf(fTheta * 0.5f);
		rotateX.arr[2][1] = -sinf(fTheta * 0.5f);
		rotateX.arr[2][2] = cosf(fTheta * 0.5f);
		rotateX.arr[3][3] = 1;
		vector<triangle> vecTrianglesToRaster;

		// drawing the triangle
		for (auto tri : meshCube.tris)
		{
			triangle triangleProjection, triTranslated, triRotatedZ, triRotatedZX, triViewed;

			// Rotate in Z-Axis
			for (int i = 0; i < 3; i++) {
				MatrixMultplication(tri.p[i], triRotatedZ.p[i], rotateZ);
			}
			// Rotate in X-Axis
			for (int i = 0; i < 3; i++) {
				MatrixMultplication(triRotatedZ.p[i], triRotatedZX.p[i], rotateX);
			}
			/////////////////////////////////////////////////////////////////////////////////////////////////////
			// Offset into the screen																	/////////
			triTranslated = triRotatedZX;																/////////
			for (int i = 0; i < 3; i++) {																/////////
				triTranslated.p[i].z = triRotatedZX.p[i].z + 8.0f; //Add to this variable to increase your field of view
			}																							/////////
			/////////////////////////////////////////////////////////////////////////////////////////////////////
			vectors3d normal, line1, line2;
			line1.x = triTranslated.p[1].x - triTranslated.p[0].x;
			line1.y = triTranslated.p[1].y - triTranslated.p[0].y;
			line1.z = triTranslated.p[1].z - triTranslated.p[0].z;

			line2.x = triTranslated.p[2].x - triTranslated.p[0].x;
			line2.y = triTranslated.p[2].y - triTranslated.p[0].y;
			line2.z = triTranslated.p[2].z - triTranslated.p[0].z;

			normal.x = line1.y * line2.z - line1.z * line2.y;
			normal.y = line1.z * line2.x - line1.x * line2.z;
			normal.z = line1.x * line2.y - line1.y * line2.x;


			float l = sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
			normal.x /= l; normal.y /= l; normal.z /= l;


			if (normal.x * (triTranslated.p[0].x - vCamera.x) + normal.y * (triTranslated.p[0].y - vCamera.y) + normal.z * (triTranslated.p[0].z - vCamera.z) < 0.0f)
			{
				// Lighting
				vectors3d light_direction = { 0.0f, 0.0f, -1.0f };
				light_direction = Vector_Normalise(light_direction);
				float dp = normal.x * light_direction.x + normal.y * light_direction.y + normal.z * light_direction.z;


				CHAR_INFO c = GetColour(dp);
				triTranslated.col = c.Attributes;
				triTranslated.sym = c.Char.UnicodeChar;
				// Project triangles from 3D --> 2D
				for (int i = 0; i < 3; i++) {
					MatrixMultplication(triTranslated.p[i], triangleProjection.p[i], matProjection);
				}
				triangleProjection.col = triTranslated.col;
				triangleProjection.sym = triTranslated.sym;

				// Scale for screen
				triangleProjection.p[0].x += 1.0f; triangleProjection.p[0].y += 1.0f;
				triangleProjection.p[1].x += 1.0f; triangleProjection.p[1].y += 1.0f;
				triangleProjection.p[2].x += 1.0f; triangleProjection.p[2].y += 1.0f;
				triangleProjection.p[0].x *= 0.5f * (float)ScreenWidth();
				triangleProjection.p[0].y *= 0.5f * (float)ScreenHeight();
				triangleProjection.p[1].x *= 0.5f * (float)ScreenWidth();
				triangleProjection.p[1].y *= 0.5f * (float)ScreenHeight();
				triangleProjection.p[2].x *= 0.5f * (float)ScreenWidth();
				triangleProjection.p[2].y *= 0.5f * (float)ScreenHeight();

			}
			vecTrianglesToRaster.push_back(triangleProjection);
		}
		// Sort triangles from back to front
		sort(vecTrianglesToRaster.begin(), vecTrianglesToRaster.end(), [](triangle& t1, triangle& t2)
			{
				float z1 = (t1.p[0].z + t1.p[1].z + t1.p[2].z) / 3.0f;
				float z2 = (t2.p[0].z + t2.p[1].z + t2.p[2].z) / 3.0f;
				return z1 > z2;
			});
		for (auto& triProjected : vecTrianglesToRaster)
		{
			FillTriangle(triProjected.p[0].x, triProjected.p[0].y, // this fills the cube with it's colour, you can comment this out to remove the white surfaces of the cube and only show the wireframe
				triProjected.p[1].x, triProjected.p[1].y,
				triProjected.p[2].x, triProjected.p[2].y,
				triProjected.sym, triProjected.col);

			DrawTriangle(triProjected.p[0].x, triProjected.p[0].y, // shows wireframe you can comment all this code out to only get a white surface and no blue wireframe
				triProjected.p[1].x, triProjected.p[1].y,
				triProjected.p[2].x, triProjected.p[2].y,
				PIXEL_SOLID, FG_BLUE); // change this FG_BLUE to FG_RED or FG_BLACK etc.. the change wireframe colour
		}
		return true;
	}

	/////////////////////////////////////////////////////////
	bool OnUserCreate() override {
		//////////////////////////////////
		// between the declaration of the function and the "projection" comment is where you can add cubes to be rendered to the screen
		// this is possible as shown below with a createCubeMesh(float x, float y, float z) function taking 3 floats, 
		// i've made it simple the function takes the x,y,z coordinates of the bottom left point of the cube and creates all the other points automatically
		// unfortunately this means you cannot scale the cube but i will implement that later. ( I will also implement more shapes later.
		// if you require more help on the usage of this code please e-mail me at manindrademel@yahoo.com.au or direct message me on instagram @mani.programming
		createCubeMesh(0, 0, 0); // this is the example of creating a cube where it's bottom left point is (0, 0, 0), try it! Run the code and you will see a cube
		createCubeMesh(0, 1, 0); // this will create another cube just one above the original, this is because it's y point is one higher.
		manualControl = false;

		// you can also change a variable called manualControl to true if you want to use the 'a' and 'd' keys to rotate around your rendered cubes manually
		////////////////////////////////// if your field of view is too small and you are unable to see all the cubes you have created please go to line 321 in the code
		// projection
		matProjection = Matrix_MakeProjection(90.0f, (float)ScreenHeight() / (float)ScreenWidth(), 0.1f, 1000.0f);
		return true;
	}
};

int main() {
	Engine instance;
	if (instance.ConstructConsole(256, 240, 4, 4)) {
		instance.Start();
	}
}