/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
//1 to turn on antialiasing
//0 for off, used for added other components to the code, anitaliasing takes long to render with
#define antialiasing 0

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
}

void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
	for (;;) {
		if (curLight == NULL) break;
		// Each lightSource provides its own shading function.
		
		
		//Check for shadows by casting a ray from the point to the source and finding first intersection
		//Create the ray from point to light
		Vector3D direction = curLight->light->get_position() - ray.intersection.point;
		Point3D origin = ray.intersection.point + 0.01*direction;
		Ray3D r(origin, direction);


		//traverse the scene and look for 
		traverseScene(_root, r);
		if (r.intersection.t_value <=1 && r.intersection.t_value>=0){
			ray.intersection.shadow = true;
		}


		
		// Implement shadows here if needed.

		curLight->light->shade(ray);
		curLight = curLight->next;
	}
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray, int depth) {
	Colour col(0.0, 0.0, 0.0); 
	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		computeShading(ray); 
		if ((*ray.intersection.mat).ref_index > 0.0 && depth < _maxDepth)
		{
			
			//Create the ray that will be reflected
			Point3D origin = ray.intersection.point;
			Vector3D direction = -(2 * (ray.dir.dot(ray.intersection.normal)*ray.intersection.normal)) 
								 + ray.dir;
			Ray3D r(origin, direction);

			//add the current colour of the ray to the colours that were reflected 
			//multiply this value by the reflection index  
			ray.col = ray.col + 
					  ray.intersection.mat->ref_index*shadeRay(r, depth + 1);
		}
		
		
			
		if (ray.intersection.mat->refractive_index > 1 && depth < _maxDepth)
		{
		///refractive index of air
			double n1, n2;
			//checks to see whether the ray is inside the material 
			if (!ray.intersection.inside){
				 n1 = 1.0;
				 n2 = ray.intersection.mat->refractive_index;
			}
			else  {
				 n2 = 1.0;
				 n1 = ray.intersection.mat->refractive_index;
			}


			//Calculate the ray that will be transmitted through the medium
			Vector3D d = ray.dir;
			Vector3D n = ray.intersection.normal;
			d.normalize();

			//t is the vector for the transmitted ray through the medium
			Vector3D t1 = n1 / n2 * (d - (d.dot(n))*n);
			double root = 1 - ((n1*n1) / (n2*n2)) * (1 - (d.dot(n)*(d.dot(n))));

			if (root < 0){} //total internal reflection do nothing
			else{
				//Finish calculating the transmitted ray equation
				Vector3D t = t1 - sqrt(root)*n;
				Point3D origin = ray.intersection.point;
				Ray3D r(origin, t);
				//assign whether the ray is going in or out of the material
				if (ray.intersection.inside)
					r.intersection.inside = false;
				else
					r.intersection.inside = true;
				ray.col = ray.col + shadeRay(r, depth + 1);
				
			}

		
			
		}	
		col = ray.col;  
		col.clamp();
	}



	
	return col;
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	_maxDepth = 5;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	if (antialiasing){
		//Anti Aliasing Implementation with a sample of 4 pixels
		int n = 2;
		// Construct a ray for each pixel.
		for (int i = 0; i < _scrHeight; i++) {
			for (int j = 0; j < _scrWidth; j++) {
				int c1 = 0, c2 = 0, c3 = 0;
				
				for (float p = 0; p <= n - 1; p++){
					for (float q = 0; q <= n - 1; q++){
						// Sets up ray origin and direction in view space, 
						// image plane is at z = -1.
						Point3D origin(0, 0, 0);
						Point3D imagePlane;
						imagePlane[0] = (-double(width) / 2 + 0.5 + i + (p + 0.5) / n) / factor;
						imagePlane[1] = (-double(height) / 2 + 0.5 + j + (q + 0.5) / n) / factor;
						imagePlane[2] = -1;


						//Convert ray to world space and create ray to be casted
						Ray3D ray;
						ray.origin = viewToWorld*origin;
						ray.dir = viewToWorld*(imagePlane - origin);
						ray.dir.normalize();
						Colour col = shadeRay(ray, 0);

						//accumulate the samples of pixels for each rgb value
						c1 = c1 + int(col[0] * 255);
						c2 = c2 + int(col[1] * 255);
						c3 = c3 + int(col[2] * 255);
					}
				}
				//divide by the n^2 value to get average colour value
				_rbuffer[i*width + j] = c1 / (n*n);
				_gbuffer[i*width + j] = c2 / (n*n);
				_bbuffer[i*width + j] = c3 / (n*n);
			}
		}
	}

	else
	{
		for (int i = 0; i < _scrHeight; i++) {
			for (int j = 0; j < _scrWidth; j++) {
				// Sets up ray origin and direction in view space, 
				// image plane is at z = -1.
				Point3D origin(0, 0, 0);
				Point3D imagePlane;
				imagePlane[0] = (-double(width) / 2 + 0.5 + j) / factor;
				imagePlane[1] = (-double(height) / 2 + 0.5 + i) / factor;
				imagePlane[2] = -1;

				// TODO: Convert ray to world space and call 
				// shadeRay(ray) to generate pixel colour. 	

				//Create rays that will be casted to build the scene
				Ray3D ray;
				ray.origin = viewToWorld*origin;
				ray.dir = viewToWorld*(imagePlane - origin);
				ray.dir.normalize();


				Colour col = shadeRay(ray,0);

				_rbuffer[i*width + j] = int(col[0] * 255);
				_gbuffer[i*width + j] = int(col[1] * 255);
				_bbuffer[i*width + j] = int(col[2] * 255);
			}
		}


	}
	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 200; 
	int height = 200; 
	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(-3, 0, -2);
	Vector3D view(1, 0, -1);
	//view.normalize();
	Vector3D up(0, 1, 0);
	double fov = 60;

	// Defines a material for shading.
	Material gold(Colour(0.3, 0.3, 0.3), Colour(0.75164, 0.60648, 0.22648),
		Colour(0.628281, 0.555802, 0.366065),
		51.2,0.0,0.0);

	Material jade(Colour(0, 0, 0), Colour(0.54, 0.89, 0.63),
		Colour(0.316228, 0.316228, 0.316228),
		12.8, 0.0,0.0);

	Material chrome(Colour(0.25,0.25,0.25), Colour(0.4, 0.4, 0.4),
		Colour(0.256777	, 0.137622	, 0.086014),
		76.8, 0.5, 1.2);



	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), 
				Colour(0.9, 0.9, 0.9) ) );

	// Add a unit square into the scene with material mat.
	SceneDagNode* cylinder = raytracer.addObject( new UnitCylinder(), &gold );
	SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );
	SceneDagNode* sphere = raytracer.addObject(new UnitSphere(), &chrome);
	
	// Apply some transformations to the unit square.
	double factor1[3] = { 1.0, 1.0, 1.0 };
	double factor2[3] = { 12.0, 12.0, 12.0 };
	raytracer.translate(sphere, Vector3D(2, -1, -5));	
	raytracer.rotate(sphere, 'x', -45); 
	raytracer.rotate(sphere, 'z', 45); 
	raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

	raytracer.translate(plane, Vector3D(0, 0, -7));	
	//raytracer.rotate(plane, 'z', 45); 
	raytracer.scale(plane, Point3D(0, 0, 0), factor2);

	raytracer.translate(cylinder, Vector3D(0, 1, -5));
	raytracer.scale(cylinder, Point3D(0,0,0), factor1);

	// Render the scene, feel free to make the image smaller for
	// testing purposes.	
	raytracer.render(width, height, eye, view, up, fov, "view1.bmp");
	
	// Render it from a different point of view.
	Point3D eye2(1, 3, -2);
	Vector3D view2(0, -1, -1);
	raytracer.render(width, height, eye2, view2, up, fov, "view2.bmp");
	
	return 0;
}

