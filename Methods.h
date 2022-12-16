#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"
#include "Matrix4.h"

Matrix4 cameraTransformation(Camera *);
Matrix4 createOrtMatrix(Camera *);
Matrix4 createPersMatrix(Camera *);
Matrix4 createViewportMatrix(Camera *);
Matrix4 modelingTransformations(Scene *, Mesh *);
void raster(Scene *,Vec4 &, Vec4 &, Vec4 &, bool );