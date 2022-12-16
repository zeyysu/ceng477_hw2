#include "Methods.h"


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
	if(u.x<=u.y){
		if(u.x<=u.z){
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
		if(u.y<=u.z){
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
	R.val[0][0] = 1;
	R.val[0][1] = 0;
	R.val[0][2] = 0;
	R.val[0][3] = 0;

	R.val[1][0] = 0;
	R.val[1][1] = cos(r->angle);
	R.val[1][2] = -1*sin(r->angle);
	R.val[1][3] = 0;

	R.val[2][0] = 0;
	R.val[2][1] = sin(r->angle);
	R.val[2][2] = cos(r->angle);
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
   	matrix.val[0][3] = -1*(camera->u.x*camera->pos.x + camera->u.y*camera->pos.y + camera->u.z*camera->pos.z);

    matrix.val[1][0] = camera->v.x;
    matrix.val[1][1] = camera->v.y;
    matrix.val[1][2] = camera->v.z;
    matrix.val[1][3] = -1*(camera->v.x*camera->pos.x + camera->v.y*camera->pos.y + camera->v.z*camera->pos.z);

    matrix.val[2][0] = camera->w.x;
    matrix.val[2][1] = camera->w.y;
    matrix.val[2][2] = camera->w.z;
    matrix.val[2][3] = -1*(camera->w.x*camera->pos.x + camera->w.y*camera->pos.y + camera->w.z*camera->pos.z);

    matrix.val[3][0] = 0.0;
    matrix.val[3][1] = 0.0;
    matrix.val[3][2] = 0.0;
    matrix.val[3][3] = 1.0;

	return matrix;

}

Matrix4 createOrtMatrix(Camera *camera){
	Matrix4 matrix;

	matrix.val[0][0] = 2/(camera->right - camera->left);
	matrix.val[0][1] = 0;
	matrix.val[0][2] = 0;
	matrix.val[0][3] = -1*(camera->right + camera->left)/(camera->right - camera->left);

	matrix.val[1][0] = 0;
	matrix.val[1][1] =  2/(camera->top - camera->bottom);
	matrix.val[1][2] = 0;
	matrix.val[1][3] = -1*(camera->top + camera->bottom)/(camera->top - camera->bottom);

	matrix.val[2][0] = 0;
	matrix.val[2][1] = 0;
	matrix.val[2][2] =  -1*2/(camera->far - camera->near);
	matrix.val[2][3] = -1*(camera->far + camera->near)/(camera->far - camera->near);

	matrix.val[3][0] = 0;
	matrix.val[3][1] = 0;
	matrix.val[3][2] = 0;
	matrix.val[3][3] = 1;

	return matrix;

}

Matrix4 createPersMatrix(Camera *camera){
	Matrix4 matrix;

	matrix.val[0][0] = (2*camera->near)/(camera->right - camera->left);
	matrix.val[0][1] = 0;
	matrix.val[0][2] = (camera->right + camera->left)/(camera->right - camera->left);
	matrix.val[0][3] = 0;

	matrix.val[1][0] = 0;
	matrix.val[1][1] =  (2*camera->near)/(camera->top - camera->bottom);
	matrix.val[1][2] = (camera->top + camera->bottom)/(camera->top - camera->bottom);
	matrix.val[1][3] = 0;

	matrix.val[2][0] = 0;
	matrix.val[2][1] = 0;
	matrix.val[2][2] =  -1*(camera->far + camera->near)/(camera->far - camera->near);
	matrix.val[2][3] = -2*(camera->far * camera->near)/(camera->far - camera->near);

	matrix.val[3][0] = 0;
	matrix.val[3][1] = 0;
	matrix.val[3][2] = -1;
	matrix.val[3][3] = 0;

	return matrix;
}

Matrix4 createViewportMatrix(Camera *camera){
	Matrix4 matrix;

	matrix.val[0][0] = camera->horRes/2;
	matrix.val[0][1] = 0;
	matrix.val[0][2] = 0;
	matrix.val[0][3] = (camera->horRes-1)/2;

	matrix.val[1][0] = 0;
	matrix.val[1][1] = camera->verRes/2;
	matrix.val[1][2] = 0;
	matrix.val[1][3] = (camera->verRes-1)/2;

	matrix.val[2][0] = 0;
	matrix.val[2][1] = 0;
	matrix.val[2][2] = 1/2;
	matrix.val[2][3] = 1/2;

	matrix.val[3][0] = 0;
	matrix.val[3][1] = 0;
	matrix.val[3][2] = 0;
	matrix.val[3][3] = 0;

	return matrix;
}


void drawLine(Scene *scene, Vec4&v1, Vec4 &v2){
	double x,y;
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
		std::cout<<"color id?1"<<std::endl;
		c = *scene->colorsOfVertices[v1.colorId-1];
		dc.r = (scene->colorsOfVertices[v2.colorId-1]->r - scene->colorsOfVertices[v1.colorId-1]->r)/(x1-x0);
		dc.g = (scene->colorsOfVertices[v2.colorId-1]->g - scene->colorsOfVertices[v1.colorId-1]->g)/(x1-x0);
		dc.b = (scene->colorsOfVertices[v2.colorId-1]->b - scene->colorsOfVertices[v1.colorId-1]->b)/(x1-x0);
		std::cout<<"color id"<<std::endl;
		}
		else{
		x0 = v2.x;
		x1 = v1.x;
		y0 = v2.y;
		y1 = v1.y;
		std::cout<<"color id?2"<<std::endl;
		c = *scene->colorsOfVertices[v2.colorId-1];
		dc.r = (scene->colorsOfVertices[v1.colorId-1]->r - scene->colorsOfVertices[v2.colorId-1]->r)/(x1-x0);
		dc.g = (scene->colorsOfVertices[v1.colorId-1]->g - scene->colorsOfVertices[v2.colorId-1]->g)/(x1-x0);
		dc.b = (scene->colorsOfVertices[v1.colorId-1]->b - scene->colorsOfVertices[v2.colorId-1]->b)/(x1-x0);
		std::cout<<"color id"<<std::endl;
		}
		y = y0;
		int dy = y0<y1? (y0-y1) : (y1-y0);
		d = 2*dy + 0.5*(x1-x0);

		for(x = x0; x <= x1; x+=1){
			std::cout<<d<<std::endl;
			std::cout<<x<<" "<<y<<std::endl;
			scene->image[round(x)][round(y)] = c;
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
			std::cout<<"color id?3 "<<v1.colorId<<" "<<scene->colorsOfVertices.size()<<std::endl;
			c = *scene->colorsOfVertices[v1.colorId-1];
			std::cout<<c<<std::endl;
			dc.r = (scene->colorsOfVertices[v2.colorId-1]->r - scene->colorsOfVertices[v1.colorId-1]->r)/(y1-y0);
			dc.g = (scene->colorsOfVertices[v2.colorId-1]->g - scene->colorsOfVertices[v1.colorId-1]->g)/(y1-y0);
			dc.b = (scene->colorsOfVertices[v2.colorId-1]->b - scene->colorsOfVertices[v1.colorId-1]->b)/(y1-y0);
			std::cout<<"color id"<<std::endl;
		}
		else{
			y0 = v2.y;
			y1 = v1.y;
			x0 = v2.x;
			x1 = v1.x;
			std::cout<<"color id?4"<<v2.colorId<<std::endl;
			c = *scene->colorsOfVertices[v2.colorId-1];
			dc.r = (scene->colorsOfVertices[v1.colorId-1]->r - scene->colorsOfVertices[v2.colorId-1]->r)/(y1-y0);
			dc.g = (scene->colorsOfVertices[v1.colorId-1]->g - scene->colorsOfVertices[v2.colorId-1]->g)/(y1-y0);
			dc.b = (scene->colorsOfVertices[v1.colorId-1]->b - scene->colorsOfVertices[v2.colorId-1]->b)/(y1-y0);
			std::cout<<"color id"<<std::endl;
		}
		x = x0;
		int dy = x0<x1? (x0-x1) : (x1-x0);
		d = 2*dy + 0.5*(y1-y0);

		for(y = y0; y <= y1; y+=1){
			std::cout<<d<<std::endl;
			std::cout<<x<<" "<<y<<std::endl;
			scene->image[round(x)][round(y)] = c;
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


void raster(Scene *scene,Vec4 &v1, Vec4 &v2, Vec4 &v3, bool solid ){
	drawLine(scene,v1,v2);
	drawLine(scene,v2,v3);
	drawLine(scene,v1,v3);

	

}

