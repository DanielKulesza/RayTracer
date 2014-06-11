/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"



bool UnitSquare::intersect(Ray3D& ray, const Matrix4x4& worldToModel,
	const Matrix4x4& modelToWorld) {

	//Find intersection in the plane that is a unit square. direction and origin are 
	//converted to model space so we can calculate normals and POI easier


	Vector3D dir = worldToModel*ray.dir;
	Point3D origin = worldToModel*ray.origin;
	//calculate the t value that makes the ray vector intersect the circle
	double t_value = (-origin[2] / dir[2]);
	Point3D point(origin[0] + t_value*dir[0], origin[1] + t_value*dir[1], 0);


	//if t value is less than zero than no intersection
	if (t_value <= 0)
		return false;


	if (-0.5 <= point[0] && point[0] <= 0.5 && -0.5 <= point[1] && point[1] <= 0.5){
		if (!ray.intersection.none && t_value > ray.intersection.t_value)
			return false;


			ray.intersection.point = modelToWorld*point;
			ray.intersection.normal = modelToWorld*Vector3D(0, 0, 1);
			//ray.intersection.normal.normalize();
			ray.intersection.none = false;
			ray.intersection.t_value = t_value;
			return true;
		
	}

	return false;
}

	

	//      return false;


bool UnitSphere::intersect(Ray3D& ray, const Matrix4x4& worldToModel,
	const Matrix4x4& modelToWorld) {
	//Find intersection 


	//origin of model space
	Point3D org(0, 0, 0);
	//Convert direction and origin to model space
	Vector3D dir = worldToModel*ray.dir;
	Point3D origin = worldToModel*ray.origin;
	//dir.normalize();

	//Sets up the parameters to solve for t_1 and t_2
	//These parameters are those for a quadratic formula
	double a = dir.dot(dir);
	double b = (origin - org).dot(dir);
	double c = (origin - org).dot(origin - org) - 1;
	//d is the descrimenant
	double d = b*b - a*c;
	double t;
	// if d is less than zero, no intersection
	if (d <= 0) {
		return false;
	}
	else if (d < 0.001)
		t = -b / a;
	// if d is around 0 then there is one solution
	else{
	// if d is positive and not 0 then evaluate as usual
		double t_1 = (-b + sqrt(d)) / a;
		double t_2 = (-b - sqrt(d)) / a;

		//Pick the t value that we will pass into the intersection parameters
		//Based on which t value produces the point closes to the ray origin
		if (t_1 < 0 && t_2 < 0)
			return false;
		if (t_1 < t_2)
			t = t_1;
		else
			t = t_2;
	}
	if (t < 0.001)
		return false;
	//check if point is already intersected
	if (!ray.intersection.none && t >= ray.intersection.t_value) {
		return false;
	}


	Point3D point = origin + t * dir;

	//Assign normal which is just the point
	Vector3D normal = 2 * (point - org);
	//normal.normalize();

	//Assign parameters to intersection
	ray.intersection.t_value = t;
	ray.intersection.point = modelToWorld * point;
	ray.intersection.normal = worldToModel * normal;
	ray.intersection.normal.normalize();
	ray.intersection.none = false;
	return true;
}


bool UnitCylinder::intersect(Ray3D& ray, const Matrix4x4& worldToModel,
	const Matrix4x4& modelToWorld) // In object coords, the cylinder has a circular base centered at x=y=0 and
	// has its axis aligned with z-axis, running from z=0 to z=+1.
{
	
		//cylinder height = 1
		//circle radius is 0.5
		//cylinder base is on x-y plane with base origin (0,0,-0.5);
		//cap origin is (0,0,0.5);
		//must be centered around the origin
		Point3D r = worldToModel * ray.origin;
		Vector3D d = worldToModel * ray.dir;
		Point3D org(0, 0, 0);
		double t1, t2,t=0;
		double a = d[0] * d[0] + d[1] * d[1];
		double b = r[0] * d[0] + r[1] * d[1];
		double c = r[0] * r[0] + r[1] * r[1] - 1;
		double D = b*b - a*c;
		Point3D point;
		Vector3D normal;
		
		
			//find the base plane t1
			t1 = (-0.5-r[2]) / d[2];

			//find the cap plane t2
			t2 = (0.5  - r[2]) / d[2];

			//find which plane is closer
			if (t1 < t2) 
			{
				t = t1;
				normal[0]=0;
				normal[1] = 0;
				normal[2] = -1;
			}
			else
			{
				t = t2;
				normal[0] = 0;
				normal[1] = 0;
				normal[2] = 1;
			}

			//find the potentinal point of intersection
			point = r + t * d;
			if (t < 0.0001)
				return false;
			
			//Check if the point is within the cap or base
			if (point[0] * point[0] + point[1] * point[1] <= 1)
			{
				//check if already intersected and closer than current t value
				if (!ray.intersection.none && t > ray.intersection.t_value)
					return false;

				//Assign values to intersection values
				ray.intersection.point =modelToWorld*point;
				ray.intersection.normal = worldToModel.transpose()*normal;
				ray.intersection.normal.normalize();
				ray.intersection.t_value = t;
				ray.intersection.none = false;
				return true;
			}

		
		//Similair procedure for sphere intersection
		if (D <= 0) 
			return false;
	
		else if (D < 0.001)
			t = -b / a;

		else{
			double t_1 = (-b + sqrt(D)) / a;
			double t_2 = (-b - sqrt(D)) / a;

			if (t_1 < 0 && t_2 < 0)
				return false;
			if (t_1 < t_2)
				t = t_1;
			else
				t = t_2;
		}


		//find potential intersection point
		point = r + t * d;
		normal = Vector3D(point[0], point[1], 0);
		if (t < 0.0001)
			return false;

		//checks the constraints 
		//If the point is in between the height parameters then we havea hit
			if (point[2] >= -0.5 && point[2] <= 0.5)
			{
				if (!ray.intersection.none && t > ray.intersection.t_value)
					return false;
				
				ray.intersection.point = modelToWorld * point;
				ray.intersection.normal = worldToModel.transpose() *normal;
				ray.intersection.normal.normalize();
				ray.intersection.t_value = t;
				ray.intersection.none = false;
				return true;
			}
		else
		{
			return false;
		}
		
}
	
	
