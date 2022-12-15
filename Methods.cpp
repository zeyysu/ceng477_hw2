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
		// std::cout<<m2<<std::endl;
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

void raster(Vec3 &v1, Vec3 &v2, Vec3 &v3, bool solid ){
	
}

