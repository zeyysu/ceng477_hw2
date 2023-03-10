#include "Methods.h"
#define PI 3.141592654

Matrix4 createTranslationMatrix(Translation *t){
	Matrix4 matrix = getIdentityMatrix();
	matrix.val[0][3] = t->tx;
	matrix.val[1][3] = t->ty;
	matrix.val[2][3] = t->tz;
	return matrix;
}

Matrix4 createScalingMatrix(Scaling *s){
	Matrix4 matrix;
	matrix.val[0][0] = s->sx;
	matrix.val[0][1] = 0;
	matrix.val[0][2] = 0;
	matrix.val[0][3] = 0;

	matrix.val[1][1] = s->sy;
	matrix.val[1][0] = 0;
	matrix.val[1][2] = 0;
	matrix.val[1][3] = 0;

	matrix.val[2][2] = s->sz;
	matrix.val[2][1] = 0;
	matrix.val[2][0] = 0;
	matrix.val[2][3] = 0;

	matrix.val[3][0] = 0;
	matrix.val[3][1] = 0;
	matrix.val[3][2] = 0;
	matrix.val[3][3] = 1;

	return matrix;
}

Matrix4 createRotationMatrix(Rotation *r){
	//alternative method
	Vec3 u;
	u.x = r->ux;
	u.y = r->uy;
	u.z = r->uz;

	u = normalizeVec3(u);
	Vec3 v;
	if(abs(u.x)<=abs(u.y)){
		if(abs(u.x)<=abs(u.z)){
			v.x = 0;
			v.y = -1*u.z;
			v.z = u.y;
		}
		else{
			v.z = 0;
			v.y = -1*u.x;
			v.x = u.y;
		}
	}
	else{
		if(abs(u.y)<=abs(u.z)){
			v.y = 0;
			v.x = -1*u.z;
			v.z = u.x;
		}
		else{
			v.z = 0;
			v.y = -1*u.x;
			v.x = u.y;
		}
	}

	v = normalizeVec3(v);
	Vec3 w = crossProductVec3(u,v);
	w = normalizeVec3(w);

	Matrix4 M;
	M.val[0][0] = u.x;
	M.val[0][1] = u.y;
	M.val[0][2] = u.z;
	M.val[0][3] = 0; 

	M.val[1][0] = v.x;
	M.val[1][1] = v.y;
	M.val[1][2] = v.z;
	M.val[1][3] = 0; 

	M.val[2][0] = w.x;
	M.val[2][1] = w.y;
	M.val[2][2] = w.z;
	M.val[2][3] = 0; 

	M.val[3][0] = 0;
	M.val[3][1] = 0;
	M.val[3][2] = 0;
	M.val[3][3] = 1;

	Matrix4 R;
	double rad = (r->angle*PI)/180;
	R.val[0][0] = 1;
	R.val[0][1] = 0;
	R.val[0][2] = 0;
	R.val[0][3] = 0;

	R.val[1][0] = 0;
	R.val[1][1] = cos(rad);
	R.val[1][2] = -1*sin(rad);
	R.val[1][3] = 0;

	R.val[2][0] = 0;
	R.val[2][1] = sin(rad);
	R.val[2][2] = cos(rad);
	R.val[2][3] = 0;

	R.val[3][0] = 0;
	R.val[3][1] = 0;
	R.val[3][2] = 0;
	R.val[3][3] = 1;

	Matrix4 M_1;
	M_1.val[0][0] = u.x;
	M_1.val[0][1] = v.x;
	M_1.val[0][2] = w.x;
	M_1.val[0][3] = 0; 

	M_1.val[1][0] = u.y;
	M_1.val[1][1] = v.y;
	M_1.val[1][2] = w.y;
	M_1.val[1][3] = 0; 

	M_1.val[2][0] = u.z;
	M_1.val[2][1] = v.z;
	M_1.val[2][2] = w.z;
	M_1.val[2][3] = 0; 

	M_1.val[3][0] = 0;
	M_1.val[3][1] = 0;
	M_1.val[3][2] = 0;
	M_1.val[3][3] = 1;


	Matrix4 matrix = multiplyMatrixWithMatrix(R,M);
	matrix = multiplyMatrixWithMatrix(M_1, matrix);
	return matrix;
	
}


Matrix4 modelingTransformations(Scene *scene, Mesh *mesh){
	int numberOfTransformations = mesh->numberOfTransformations;
	int i;
	Matrix4 matrix = getIdentityMatrix();
	Matrix4 m2;
	for(i=0;i<numberOfTransformations;i++){
		switch(mesh->transformationTypes[i]){
			case 't':
				m2 = createTranslationMatrix(scene->translations[mesh->transformationIds[i]-1]);
				matrix = multiplyMatrixWithMatrix(m2, matrix);
				break;
			case 's':
				m2 = createScalingMatrix(scene->scalings[mesh->transformationIds[i]-1]);
				matrix = multiplyMatrixWithMatrix(m2, matrix);
				break;
			case 'r':
				m2 = createRotationMatrix(scene->rotations[mesh->transformationIds[i]-1]);
				matrix = multiplyMatrixWithMatrix(m2, matrix);
				break;
			default:
				break;
		}
	}
	return matrix;
};

Matrix4 cameraTransformation(Camera *camera){
	Matrix4 matrix;
	matrix.val[0][0] = camera->u.x;
    matrix.val[0][1] = camera->u.y;
    matrix.val[0][2] = camera->u.z;
   	matrix.val[0][3] = (-1.0)*(camera->u.x*camera->pos.x + camera->u.y*camera->pos.y + camera->u.z*camera->pos.z);

    matrix.val[1][0] = camera->v.x;
    matrix.val[1][1] = camera->v.y;
    matrix.val[1][2] = camera->v.z;
    matrix.val[1][3] = (-1.0)*(camera->v.x*camera->pos.x + camera->v.y*camera->pos.y + camera->v.z*camera->pos.z);

    matrix.val[2][0] = camera->w.x;
    matrix.val[2][1] = camera->w.y;
    matrix.val[2][2] = camera->w.z;
    matrix.val[2][3] = (-1.0)*(camera->w.x*camera->pos.x + camera->w.y*camera->pos.y + camera->w.z*camera->pos.z);

    matrix.val[3][0] = 0.0;
    matrix.val[3][1] = 0.0;
    matrix.val[3][2] = 0.0;
    matrix.val[3][3] = 1.0;

	return matrix;

}

Matrix4 createOrtMatrix(Camera *camera){
	Matrix4 matrix;

	matrix.val[0][0] = 2.0/(camera->right - camera->left);
	matrix.val[0][1] = 0;
	matrix.val[0][2] = 0;
	matrix.val[0][3] = (-1.0)*(camera->right + camera->left)/(camera->right - camera->left);

	matrix.val[1][0] = 0;
	matrix.val[1][1] =  2.0/(camera->top - camera->bottom);
	matrix.val[1][2] = 0;
	matrix.val[1][3] = (-1.0)*(camera->top + camera->bottom)/(camera->top - camera->bottom);

	matrix.val[2][0] = 0;
	matrix.val[2][1] = 0;
	matrix.val[2][2] =  (-2.0)/(camera->far - camera->near);
	matrix.val[2][3] = (-1.0)*(camera->far + camera->near)/(camera->far - camera->near);

	matrix.val[3][0] = 0;
	matrix.val[3][1] = 0;
	matrix.val[3][2] = 0;
	matrix.val[3][3] = 1.0;

	return matrix;

}

Matrix4 createPersMatrix(Camera *camera){
	Matrix4 matrix;

	matrix.val[0][0] = (2.0*camera->near)/(camera->right - camera->left);
	matrix.val[0][1] = 0;
	matrix.val[0][2] = (camera->right + camera->left)/(camera->right - camera->left);
	matrix.val[0][3] = 0;

	matrix.val[1][0] = 0;
	matrix.val[1][1] =  (2.0*camera->near)/(camera->top - camera->bottom);
	matrix.val[1][2] = (camera->top + camera->bottom)/(camera->top - camera->bottom);
	matrix.val[1][3] = 0;

	matrix.val[2][0] = 0;
	matrix.val[2][1] = 0;
	matrix.val[2][2] =  (-1.0)*(camera->far + camera->near)/(camera->far - camera->near);
	matrix.val[2][3] = (-2.0)*(camera->far * camera->near)/(camera->far - camera->near);

	matrix.val[3][0] = 0;
	matrix.val[3][1] = 0;
	matrix.val[3][2] = -1.0;
	matrix.val[3][3] = 0;

	return matrix;
}

Matrix4 createViewportMatrix(Camera *camera){
	Matrix4 matrix;

	matrix.val[0][0] = camera->horRes/2.0;
	matrix.val[0][1] = 0;
	matrix.val[0][2] = 0;
	matrix.val[0][3] = (camera->horRes-1)/2.0;

	matrix.val[1][0] = 0;
	matrix.val[1][1] = camera->verRes/2.0;
	matrix.val[1][2] = 0;
	matrix.val[1][3] = (camera->verRes-1)/2.0;

	matrix.val[2][0] = 0;
	matrix.val[2][1] = 0;
	matrix.val[2][2] = 0.5;
	matrix.val[2][3] = 0.5;

	matrix.val[3][0] = 0;
	matrix.val[3][1] = 0;
	matrix.val[3][2] = 0;
	matrix.val[3][3] = 0;

	return matrix;
}


void drawLine(Scene *scene, Vec4&v1, Vec4 &v2, Camera *camera){
	int x,y;
	double d;
	double y0,y1;
	double x0,x1;
	Color c;
	Color dc;
	double slope = 0;
	if(x1!=x0) slope = abs(v1.y-v2.y)/abs(v1.x-v2.x);
	if(slope<=1){
		if( v1.x <= v2.x ){
		x0 = v1.x;
		x1 = v2.x;
		y0 = v1.y;
		y1 = v2.y;
		c = *scene->colorsOfVertices[v1.colorId-1];
		dc.r = (scene->colorsOfVertices[v2.colorId-1]->r - scene->colorsOfVertices[v1.colorId-1]->r)/(x1-x0);
		dc.g = (scene->colorsOfVertices[v2.colorId-1]->g - scene->colorsOfVertices[v1.colorId-1]->g)/(x1-x0);
		dc.b = (scene->colorsOfVertices[v2.colorId-1]->b - scene->colorsOfVertices[v1.colorId-1]->b)/(x1-x0);
		}
		else{
		x0 = v2.x;
		x1 = v1.x;
		y0 = v2.y;
		y1 = v1.y;
		c = *scene->colorsOfVertices[v2.colorId-1];
		dc.r = (scene->colorsOfVertices[v1.colorId-1]->r - scene->colorsOfVertices[v2.colorId-1]->r)/(x1-x0);
		dc.g = (scene->colorsOfVertices[v1.colorId-1]->g - scene->colorsOfVertices[v2.colorId-1]->g)/(x1-x0);
		dc.b = (scene->colorsOfVertices[v1.colorId-1]->b - scene->colorsOfVertices[v2.colorId-1]->b)/(x1-x0);
		}
		y = y0;
		int dy = y0<y1? (y0-y1) : (y1-y0);
		d = 2*dy + 0.5*(x1-x0);

		for(x = x0; x <= x1; x+=1){
			if(x<0) continue;
			if(y<0){
				if(y0<y1) {y+=1; continue;}
				else break;
			}
			if(x >= camera->horRes) break;
			if(y >= (camera->verRes)){
				if(y0<y1) break;
				else y-=1;
				continue;
			}
			scene->image[x][y] = c;
			if(d<0){
				if(y0<y1) y+=1;
				else y-=1;
				d+= 2*(dy + (x1-x0));
			}
			else{
				d+= 2*dy;
			}
			c.r += dc.r;
			c.g += dc.g;
			c.b += dc.b;
		}
	}
	else{
		if( v1.y <= v2.y ){
			y0 = v1.y;
			y1 = v2.y;
			x0 = v1.x;
			x1 = v2.x;
			c = *scene->colorsOfVertices[v1.colorId-1];
			dc.r = (scene->colorsOfVertices[v2.colorId-1]->r - scene->colorsOfVertices[v1.colorId-1]->r)/(y1-y0);
			dc.g = (scene->colorsOfVertices[v2.colorId-1]->g - scene->colorsOfVertices[v1.colorId-1]->g)/(y1-y0);
			dc.b = (scene->colorsOfVertices[v2.colorId-1]->b - scene->colorsOfVertices[v1.colorId-1]->b)/(y1-y0);
		}
		else{
			y0 = v2.y;
			y1 = v1.y;
			x0 = v2.x;
			x1 = v1.x;
			c = *scene->colorsOfVertices[v2.colorId-1];
			dc.r = (scene->colorsOfVertices[v1.colorId-1]->r - scene->colorsOfVertices[v2.colorId-1]->r)/(y1-y0);
			dc.g = (scene->colorsOfVertices[v1.colorId-1]->g - scene->colorsOfVertices[v2.colorId-1]->g)/(y1-y0);
			dc.b = (scene->colorsOfVertices[v1.colorId-1]->b - scene->colorsOfVertices[v2.colorId-1]->b)/(y1-y0);
		}
		x = x0;
		int dy = x0<x1? (x0-x1) : (x1-x0);
		d = 2*dy + 0.5*(y1-y0);

		for(y = y0; y <= y1; y+=1){
			if(y<0) continue;
			if(x<0){
				if(x0<x1) {x+=1; continue;}
				else break;
			}
			if(y >= camera->verRes) break;
			if(x >= (camera->horRes)){
				if(x0<x1) break;
				else x-=1;
				continue;
			}
			
			scene->image[x][y] = c;
			if(d<0){
				if(x0<x1) x+=1;
				else x-=1;
				d+= 2*(dy + (y1-y0));
			}
			else{
				d+= 2*dy;
			}
			c.r += dc.r;
			c.g += dc.g;
			c.b += dc.b;
		}
	}
	
}

double f(double x, double y, double x_0, double y_0, double x_1, double y_1){
    return x * (y_0 - y_1) + y * (x_1 - x_0) + (x_0 * y_1) - (y_0 * x_1);
}


void triangle_raster(Scene *scene, Vec4 &v0, Vec4 &v1, Vec4 &v2, Camera *camera){
	double xmin = min(v1.x, v2.x);
	xmin = min(xmin, v0.x);
	xmin= round(xmin);
	double ymin =  min(v1.y, v2.y);
	ymin = min(ymin, v0.y);
	ymin = round(ymin);
	double xmax = max(v1.x, v2.x);
	xmax = max(xmax, v0.x);
	xmax = round(xmax);
	double ymax = max(v1.y, v2.y);
	ymax = max(ymax, v0.y);
	ymax = round(ymax);
	Color c0 =  *(scene->colorsOfVertices[v0.colorId-1]);
	Color c1 =  *(scene->colorsOfVertices[v1.colorId-1]);
	Color c2 =  *(scene->colorsOfVertices[v2.colorId-1]);
	Color c;
	for(int x=xmin; x<=xmax; x++){
		if(x<0) continue;
		if(x>=camera->horRes) break;
		for(int y=ymin; y<=ymax; y++){
			if(y<0) continue;
			if(y>=camera->verRes) break;
			double alpha = f(x, y, v1.x, v1.y, v2.x, v2.y) / f(v0.x, v0.y, v1.x ,v1.y,v2.x,v2.y);
			double beta = f(x, y ,v2.x ,v2.y,v0.x,v0.y) /  f(v1.x, v1.y ,v2.x ,v2.y,v0.x,v0.y);
			double gama =  f(x, y ,v0.x ,v0.y,v1.x,v1.y) / f(v2.x, v2.y ,v0.x ,v0.y,v1.x,v1.y);
			if(alpha>=0 & beta >=0 && gama>=0){
			c.r = alpha*c0.r + beta*c1.r + gama*c2.r;
			c.g = alpha*c0.g + beta*c1.g + gama*c2.g;
			c.b = alpha*c0.b + beta*c1.b + gama*c2.b;
			scene->image[x][y] = c;
			}
		}
	}

}

bool backfaceCulling(Vec4 & v1, Vec4 &v2, Vec4& v3 ){
	Vec3 v1_t;
	Vec3 v2_t;
	Vec3 v3_t;
	
	v1_t.x = v1.x; v1_t.y=v1.y; v1_t.z= v1.z; v1_t.colorId = v1.colorId;
	v2_t.x = v2.x; v2_t.y=v2.y; v2_t.z= v2.z; v2_t.colorId = v2.colorId;
	v3_t.x = v3.x; v3_t.y=v3.y; v3_t.z= v3.z; v3_t.colorId = v3.colorId;
	Vec3 v1_v2 = subtractVec3(v2_t, v1_t);
	Vec3 v1_v3 = subtractVec3(v3_t, v1_t);
	Vec3 normal = crossProductVec3(v1_v2, v1_v3);
	double dot = dotProductVec3(normal, v1_t);
	if(dot < 0) return true;
	return false;

}

bool isVisible(double p, double q, double & tE, double & tL) {

		double t = q / p;
		if (p > 0) {
			if (t > tL) {
				return false;
			} else if (t > tE) {
				tE = t;
			}
		} else if(p < 0) {
			if (t < tE) {
				return false;
			} else if (t < tL) {
				tL = t;
			}
		}
		else if (q > 0) {
			return false;
		}
	
	return true;
}


bool clippedLine(Scene *scene,Vec4 & v1_t, Vec4 & v2_t, Color & c1, Color & c2) {
    bool result = false;
    Vec4 v0 = v1_t, v1 = v2_t;
	int bflag = 0;
    double tE = 0, tL = 1;
	double d[]= {v1.x-v0.x,v1.y-v0.y,v1.z-v0.z};
	double  min[] = {-1,-1,-1};
	double max[] = {1,1,1};
	double * v[] = {&v0.x,&v0.y,&v0.z};
	Color dcolor;
	dcolor.r = c2.r - c1.r;
	dcolor.g = c2.g - c1.g;
	dcolor.b = c2.b - c1.b;
	Color *c1_t =new Color(c1);
	Color *c2_t = new Color(c2);
	int newColorId1 = v0.colorId;
	int newColorId2 = v1.colorId;

	
	
	for (int i = 0; i < 3; i++) {
		if (isVisible(d[i], min[i]-(*v[i]), tE, tL) && isVisible(-d[i], (*v[i])-max[i], tE, tL)) {
			bflag ++;
		}
	}
		if(bflag == 3){
			result = true;
			/* Line is visible */
			if (tL < 1.0) {
				v1.x = v0.x + (d[0] * tL);
				v1.y = v0.y + (d[1] * tL);
				v1.z = v0.z + (d[2] * tL);
				c2_t->r = c1_t->r + (dcolor.r * tL);
				c2_t->g = c1_t->g + (dcolor.g * tL);
				c2_t->b = c1_t->b + (dcolor.b * tL);
				scene->colorsOfVertices.push_back(c2_t);
				newColorId2= scene->colorsOfVertices.size();

			}
			if (tE > 0.0) {
				v0.x = v0.x + (d[0] * tE);
				v0.y = v0.y + (d[1] * tE);
				v0.z = v0.z + (d[2] * tE);
				c1_t->r = c1_t->r + (dcolor.r * tE);
				c1_t->g = c1_t->g + (dcolor.g * tE);
				c1_t->b = c1_t->b + (dcolor.b * tE);
				scene->colorsOfVertices.push_back(c1_t);
				newColorId1= scene->colorsOfVertices.size();
			}
		}

	v1_t = v0;
	v1_t.colorId = newColorId1;
	v2_t = v1;
	v2_t.colorId = newColorId2;
	return result;
}