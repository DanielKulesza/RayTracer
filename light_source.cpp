/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

void PointLight::shade( Ray3D& ray ) {
  
	
	//Assign K colour values, specular exponent and light colours to variables
	Colour Ka = (*ray.intersection.mat).ambient;
	Colour Kd = (*ray.intersection.mat).diffuse;
	Colour Ks = (*ray.intersection.mat).specular;
	double alpha = (*ray.intersection.mat).specular_exp;
	Colour Id = _col_diffuse;
	Colour Ia = _col_ambient;
	Colour Is = _col_specular;

	//assign normal and then normalize
	Vector3D normal = ray.intersection.normal;
	normal.normalize();

	//set up light direction from intersection point
	Vector3D lightDirection = _pos - ray.intersection.point;
	lightDirection.normalize();

	//opposite of ray direction is 
	Vector3D v = -ray.dir;
	//v.normalize();

	//
	Vector3D r = 2 * (lightDirection.dot(normal))*normal - lightDirection;
	r.normalize();

	Colour col;
	//calculate the ambient, diffuse and specular components of phong shading
	Colour ambient = Ka*Ia;
	Colour diffuse = Kd*Id*fmax(0, normal.dot(lightDirection));
	Colour specular = Ks*Is*(fmax(0, pow((v.dot(r)), alpha)));
	//checks for shadows and assigns only ambient if the pixel is indeed to
	//be coloured as a shadow
	if (ray.intersection.shadow)
		col = ambient;
	else
		col = ambient + diffuse + specular;
	//calculate and assign the phong shading to colour
	ray.col = ray.col + col;
	ray.col.clamp();
	
}

