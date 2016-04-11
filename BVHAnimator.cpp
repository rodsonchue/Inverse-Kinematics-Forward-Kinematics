/* CS3242 3D Modeling and Animation
 * Programming Assignment II
 * School of Computing
 * National University of Singapore
 */
 
#include "BVHAnimator.h"

#include <ctime>

#include <iostream>
#include <queue>
using namespace std;

// color used for rendering, in RGB
float color[3];

// convert a matrix to an array
static void mat4ToGLdouble16(GLdouble* m, glm::mat4 mat){
	// OpenGL matrix is column-major.
	for (uint i = 0; i < 4; i++)
		for (uint j = 0; j < 4; j++)
			m[i*4+j] = mat[i][j];
}

static glm::mat4 rigidToMat4(RigidTransform t) {
    glm::mat4 translation_mat = glm::translate(glm::mat4(1.0f), t.translation);
    glm::mat4 rotation_mat = glm::mat4_cast(t.quaternion);
    return translation_mat * rotation_mat;
}

static void rigidToGLdouble16(GLdouble *m, RigidTransform t) {
    mat4ToGLdouble16(m, rigidToMat4(t));
}

void renderSphere( float x, float y, float z, float size){		
	float radius = size;
	int numSlices = 32;
	int numStacks = 8; 
	static GLUquadricObj *quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();	
	glTranslated( x, y, z );

	glColor3f(color[0],color[1],color[2]);

	gluSphere(quad_obj,radius,numSlices,numStacks);	

	glPopMatrix();
}

/**
 * Draw a bone from (x0,y0,z0) to (x1,y1,z1) as a cylinder of radius (boneSize)
 */
void renderBone( float x0, float y0, float z0, float x1, float y1, float z1, float boneSize ){
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );

	static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();	
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();

	glTranslated( x0, y0, z0 );

	double  length;
	length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	if ( length < 0.0001 ) { 
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;

	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;

	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;

	length = sqrt( side_x*side_x + side_y*side_y + side_z*side_z );
	if ( length < 0.0001 ) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;

	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
	                    up_x,   up_y,   up_z,   0.0,
	                    dir_x,  dir_y,  dir_z,  0.0,
	                    0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd( m );

	GLdouble radius= boneSize; 
	GLdouble slices = 8.0; 
	GLdouble stack = 3.0;  
	glColor3f(color[0],color[1],color[2]);
	gluCylinder( quad_obj, radius, radius, bone_length, slices, stack ); 

	glPopMatrix();
}

BVHAnimator::BVHAnimator(BVH* bvh)
{
	_bvh = bvh;
	setPointers();
}

void  BVHAnimator::renderFigure( int frame_no, float scale, int flag )
{	
	switch (flag){
	case 0:
		renderJointsGL( _bvh->getRootJoint(), _bvh->getMotionDataPtr(frame_no), scale);
	    break;
    case 1:
        renderJointsMatrix(frame_no, scale);
        break;
    case 2:
		renderJointsQuaternion(frame_no, scale);
		break;
	case 3:
		renderSkeleton( _bvh->getRootJoint(), _bvh->getMotionDataPtr(frame_no), scale );	
		break;
	case 4:	
		renderMannequin(frame_no,scale);
		break;
	default:
		break;
	}
}

void  BVHAnimator::renderSkeleton( const JOINT* joint, const float*data, float scale )
{	
	color[0] = 1.;
    color[1] = 0.;
    color[2] = 0.;

	float bonesize = 0.03;
	
    glPushMatrix();
	
    // translate
	if ( joint->parent == NULL )    // root
	{
		glTranslatef( data[0] * scale, data[1] * scale, data[2] * scale );
	}
	else
	{
		glTranslatef( joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale );
	}

	// rotate
	for ( uint i=0; i<joint->channels.size(); i++ )
	{
		CHANNEL *channel = joint->channels[i];
		if ( channel->type == X_ROTATION )
			glRotatef( data[channel->index], 1.0f, 0.0f, 0.0f );
		else if ( channel->type == Y_ROTATION )
			glRotatef( data[channel->index], 0.0f, 1.0f, 0.0f );
		else if ( channel->type == Z_ROTATION )
			glRotatef( data[channel->index], 0.0f, 0.0f, 1.0f );
	}

	//end site? skip!
	if ( joint->children.size() == 0 && !joint->is_site)
	{
		renderBone(0.0f, 0.0f, 0.0f, joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale,bonesize);
	}
	if ( joint->children.size() == 1 )
	{
		JOINT *  child = joint->children[ 0 ];
		renderBone(0.0f, 0.0f, 0.0f, child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,bonesize);
	}
	if ( joint->children.size() > 1 )
	{
		float  center[ 3 ] = { 0.0f, 0.0f, 0.0f };
		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			center[0] += child->offset.x;
			center[1] += child->offset.y;
			center[2] += child->offset.z;
		}
		center[0] /= joint->children.size() + 1;
		center[1] /= joint->children.size() + 1;
		center[2] /= joint->children.size() + 1;

		renderBone(	0.0f, 0.0f, 0.0f, center[0]*scale, center[1]*scale, center[2]*scale,bonesize);

		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			renderBone(	center[0]*scale, center[1]*scale, center[2]*scale,
				child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,bonesize);
		}
	}

	// recursively render all bones
	for ( uint i=0; i<joint->children.size(); i++ )
	{
		renderSkeleton( joint->children[ i ], data, scale );
	}
	glPopMatrix();
}

void  BVHAnimator::renderJointsGL( const JOINT* joint, const float*data, float scale )
{	
    // --------------------------------------
    // [Part 2a - Forward Kinematics]
    // --------------------------------------
    
	color[0] = 1.; color[1] = 0.; color[2] = 0.;

	glPushMatrix();

	// translate
	if ( joint->parent == NULL )    // root
	{
		glTranslatef( data[0] * scale, data[1] * scale, data[2] * scale );
	}
	else
	{
		glTranslatef( joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale );
	}

	// rotate
	for (uint i = 0; i<joint->channels.size(); i++)
	{
		CHANNEL *channel = joint->channels[i];
		if (channel->type == X_ROTATION)
			glRotatef(data[channel->index], 1.0f, 0.0f, 0.0f);
		else if (channel->type == Y_ROTATION)
			glRotatef(data[channel->index], 0.0f, 1.0f, 0.0f);
		else if (channel->type == Z_ROTATION)
			glRotatef(data[channel->index], 0.0f, 0.0f, 1.0f);
	}

	// end site
	if ( joint->children.size() == 0 )
	{
		renderSphere(joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale,0.07);
	}
	if ( joint->children.size() == 1 )
	{
		JOINT *  child = joint->children[ 0 ];
		renderSphere(child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,0.07);
	}
	if ( joint->children.size() > 1 )
	{
		float  center[ 3 ] = { 0.0f, 0.0f, 0.0f };
		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			center[0] += child->offset.x;
			center[1] += child->offset.y;
			center[2] += child->offset.z;
		}
		center[0] /= joint->children.size() + 1;
		center[1] /= joint->children.size() + 1;
		center[2] /= joint->children.size() + 1;

		renderSphere(center[0]*scale, center[1]*scale, center[2]*scale,0.07);

		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			renderSphere(child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,0.07);
		}
	}

	// recursively render all joints
	for ( uint i=0; i<joint->children.size(); i++ )
	{
		renderJointsGL( joint->children[i], data, scale );
	}
	glPopMatrix();
}


void  BVHAnimator::renderJointsMatrix( int frame, float scale )
{
	_bvh->matrixMoveTo(frame, scale);
	std::vector<JOINT*> jointList = _bvh->getJointList();	
	for(std::vector<JOINT*>::iterator it = jointList.begin(); it != jointList.end(); it++)
	{
		glPushMatrix();	
                
        GLdouble m[16];                  
		mat4ToGLdouble16(m, (*it)->matrix);
		glMultMatrixd(m);

		if ((*it)->is_site) {
			color[0] = 0.; color[1] = 1.; color[2] = 0.;
			renderSphere(0, 0, 0, 0.04);
		}
		else{
			color[0] = 0.; color[1] = 1.; color[2] = 0.;
			renderSphere(0, 0, 0, 0.07);
		}

		glPopMatrix();
	}	
}

void  BVHAnimator::renderJointsQuaternion( int frame, float scale )
{
	_bvh->quaternionMoveTo(frame, scale);
	std::vector<JOINT*> jointList = _bvh->getJointList();	
	for(std::vector<JOINT*>::iterator it = jointList.begin(); it != jointList.end(); it++)
	{
		glPushMatrix();	

        // convert quaternion and translation into matrix for rendering        
        glm::mat4 mat = rigidToMat4((*it)->transform);
        GLdouble m[16];                  
		mat4ToGLdouble16(m, mat);
		glMultMatrixd(m);

		if ((*it)->is_site) {
			color[0] = 0.; color[1] = 0.; color[2] = 1.;
			renderSphere(0, 0, 0, 0.04);
		} else {
			color[0] = 0.; color[1] = 0.; color[2] = 1.;
			renderSphere(0, 0, 0, 0.07);
		}

		glPopMatrix();
	}	
}

void BVHAnimator::setPointers(){
	head = _bvh->getJoint(std::string(_BVH_HEAD_JOINT_));
	neck = _bvh->getJoint(std::string(_BVH_NECK_JOINT_));
	chest = _bvh->getJoint(std::string(_BVH_CHEST_JOINT_));
	spine = _bvh->getJoint(std::string(_BVH_SPINE_JOINT_));
	hip = _bvh->getJoint(std::string(_BVH_ROOT_JOINT_));   // root joint

	lshldr = _bvh->getJoint(std::string(_BVH_L_SHOULDER_JOINT_));
	larm = _bvh->getJoint(std::string(_BVH_L_ARM_JOINT_));
	lforearm = _bvh->getJoint(std::string(_BVH_L_FOREARM_JOINT_));
	lhand = _bvh->getJoint(std::string(_BVH_L_HAND_JOINT_));

	rshldr = _bvh->getJoint(std::string(_BVH_R_SHOULDER_JOINT_));
	rarm = _bvh->getJoint(std::string( _BVH_R_ARM_JOINT_));
	rforearm = _bvh->getJoint(std::string(_BVH_R_FOREARM_JOINT_));
	rhand = _bvh->getJoint(std::string(_BVH_R_HAND_JOINT_));

	lupleg = _bvh->getJoint(std::string(_BVH_L_THIGH_JOINT_));
	lleg = _bvh->getJoint(std::string(_BVH_L_SHIN_JOINT_));
	lfoot = _bvh->getJoint(std::string(_BVH_L_FOOT_JOINT_));
	ltoe = _bvh->getJoint(std::string(_BVH_L_TOE_JOINT_));

	rupleg = _bvh->getJoint(std::string(_BVH_R_THIGH_JOINT_));
	rleg = _bvh->getJoint(std::string(_BVH_R_SHIN_JOINT_));
	rfoot = _bvh->getJoint(std::string(_BVH_R_FOOT_JOINT_));
	rtoe = _bvh->getJoint(std::string(_BVH_R_TOE_JOINT_));
}

void getScaledOffset(OFFSET& c, OFFSET a, OFFSET b, float scale)
{
	c.x = (a.x-b.x)*scale;
	c.y = (a.y-b.y)*scale;
	c.z = (a.z-b.z)*scale;
}

void BVHAnimator::renderMannequin(int frame, float scale) {

    // --------------------------------------
    // [Part 2c - Forward Kinematics]
    // --------------------------------------
	// You can draw a couple of basic geometries to build the mannequin 
    // using the renderSphere() and renderBone() provided in BVHAnimator.cpp 
    // or GL functions like glutSolidCube(), etc.
    
    //_bvh->quaternionMoveTo(frame, scale);

    _bvh->matrixMoveTo(frame, scale);
    // Using matrix to calculate

	std::vector<JOINT*> jointList = _bvh->getJointList();
	for (std::vector<JOINT*>::iterator it = jointList.begin(); it != jointList.end(); it++)
	{
		glPushMatrix();

		GLdouble m[16];
		mat4ToGLdouble16(m, (*it)->matrix);
		glMultMatrixd(m);

		//Applies to all primitives drawn for consistency
		float thickness = 0.1;
		float sphereThickness = thickness * 1.2;

		//If endpoint, only draw sphere
		color[0] = 1.; color[1] = 0.; color[2] = 0.;
		renderSphere(0, 0, 0, sphereThickness); //slightly more for distinction

		//If not an endpoint, also draw bones going out towards each child joint
		if (!(*it)->is_site) {

			//Only needs to be redefined once for each loop
			color[0] = 0.; color[1] = 1.; color[2] = 0.;
			for (std::vector<JOINT*>::iterator cit = (*it)->children.begin();
				 cit != (*it)->children.end(); cit++) {

				renderBone(0., 0., 0.,
					(*cit)->offset.x * scale, (*cit)->offset.y * scale, (*cit)->offset.z * scale,
					thickness);
			}
		}

		glPopMatrix();
	}

}

void BVHAnimator::solveLeftArm(int frame_no, float scale, float x, float y, float z)
{
    //_bvh->matrixMoveTo(frame_no, scale);      
    _bvh->quaternionMoveTo(frame_no, scale);
    // NOTE: you can use either matrix or quaternion to calculate the transformation

	float *LArx, *LAry, *LArz, *LFAry;
	
	float *mdata = _bvh->getMotionDataPtr(frame_no);
	// 3 channels - Xrotation, Yrotation, Zrotation
    // extract value address from motion data        
    CHANNEL *channel = larm->channels[0];
	LArx = &mdata[channel->index];
	channel = larm->channels[1];
	LAry = &mdata[channel->index];
	channel = larm->channels[2];
	LArz = &mdata[channel->index];

	channel = lforearm->channels[1];
	LFAry = &mdata[channel->index];
    
    cout << "Solving inverse kinematics..." << endl;
    clock_t start_time = clock();

    // -------------------------------------------------------
    // TODO: [Part 3] - Inverse Kinematics
    //
    // Put your code below
    // -------------------------------------------------------
	glm::vec3 target(x, y, z);
	bool isNotGoodEnough = true;
	int cycles = 0;
	float errorMargin = 0.0001;

	//p0, the base position of the shoulder (base of the IK)
	glm::vec3 p0 = lshldr->transform.translation;
	//The additional quaternion to apply for this joint
	glm::quat p0quat = glm::quat();

	//p1, the position of the left arm joint
	glm::vec3 p1 = larm->transform.translation;
	//The additional quaternion to apply for this joint
	glm::quat p1quat = glm::quat();

	//p2, the position of the left forearm joint
	glm::vec3 p2 = lforearm->transform.translation;
	//p3, the position of the end point (on the hand)
	glm::vec3 p3 = lhand->transform.translation;

	//K is the length of a vector
	float k_cos_theta;
	float k;
	//The additional rotation to perform for any given orientation
	glm::quat rotationQuat;

	//If error is trivial enough, we assume it is already "good enough"
	if (p3.x - x < errorMargin &&
		p3.y - y < errorMargin && 
		p3.x - x < errorMargin) {
		isNotGoodEnough = false;
	}
	

	//Try to find a good orientation for all involved joints
	while (isNotGoodEnough || cycles > 1000) {
		for (uint j = 0; j <= 1 && isNotGoodEnough; j++) {
			JOINT* joint;
			glm::vec3 p;
			if (j == 0) {
				//We rotate elbow
				joint = larm;
				p = p1;
			}
			else { //j == 1
				//We rotate shoulder
				joint = lshldr;
				p = p0;
			}

			//////////////////////////////////////////////////////////////////////////////////////
			//General steps to apply a quaternion to a joint and update its children as follows.

			//Step 1: Get a quaternion that rotates the joint towards end point
			//v, the normalized vector from p to the "current" end point
			glm::vec3 v = glm::normalize(p3 - p);
			//t, the normalized vector from p to the "target" end point
			glm::vec3 t = glm::normalize(target - p);
			//Compute quaternion to apply to reach target point (as much as possible)
			k_cos_theta = glm::dot(v, t);
			k = glm::sqrt(glm::length(v) * glm::length(v)
				* glm::length(t) * glm::length(t));
			rotationQuat = glm::quat(k_cos_theta + k, glm::cross(v, t));

			//Fore arm has only 1DOF (in Y axis), so we must remove the other degrees from quat
			if (j == 0) {
				rotationQuat.x = 0;
				rotationQuat.z = 0;
			}

			rotationQuat = glm::normalize(rotationQuat);
			if (j == 0) {
				p0quat = p0quat * rotationQuat;
			}
			else {
				p1quat = p1quat * rotationQuat;
			}

			//Step 2: Update joint data for itself and all its children
			//If we process the joints in FIFO, we guarentee the updates are correct
			//i.e child joints will not be processed until their parents are
			std::queue<JOINT*> jointQueue;
			jointQueue.push(joint);

			while (!jointQueue.empty()) {
				JOINT* joint = jointQueue.front();
				jointQueue.pop();

				// extract value from motion data  (again)
				// translate to the offset and set the rotation to identity
				joint->transform.translation = glm::vec3(joint->offset.x * scale,
					joint->offset.y * scale,
					joint->offset.z * scale);
				joint->transform.quaternion = glm::quat();

				RigidTransform rt;
				rt.quaternion = glm::quat();
				rt.translation = glm::vec3(joint->offset.x * scale,
					joint->offset.y * scale,
					joint->offset.z * scale);

				for (uint i = 0; i < joint->channels.size(); i++)
				{
					CHANNEL *channel = joint->channels[i];
					float value = mdata[channel->index];
					switch (channel->type) {
					case X_POSITION:
						rt.translation.x += value * scale;
						break;

					case Y_POSITION:
						rt.translation.y += value * scale;
						break;

					case Z_POSITION:
						rt.translation.z += value * scale;
						break;

					case X_ROTATION:
					{
						rt.quaternion = rt.quaternion
							* glm::angleAxis(value, glm::vec3(1.f, 0.f, 0.f));
						break;
					}
					case Y_ROTATION:
					{
						rt.quaternion = rt.quaternion
							* glm::angleAxis(value, glm::vec3(0.f, 1.f, 0.f));
						break;
					}
					case Z_ROTATION:
					{
						rt.quaternion = rt.quaternion
							* glm::angleAxis(value, glm::vec3(0.f, 0.f, 1.f));
						break;
					}
					}

					if (joint == lshldr) {
						//Its translation does not change
						//inherit the parent's orientation
						joint->transform.quaternion =
							glm::cross(joint->parent->transform.quaternion, rt.quaternion);
						//and then add the additional rotation
						joint->transform.quaternion =
							glm::cross(joint->transform.quaternion, p0quat);
					}
					else if (joint == larm) {
						//Update its translation
						joint->transform.translation = joint->parent->transform.translation
							+ (joint->parent->transform.quaternion * rt.translation);

						//inherit the parent's orientation
						joint->transform.quaternion =
							glm::cross(joint->parent->transform.quaternion, rt.quaternion);
						//and then add the additional rotation
						joint->transform.quaternion =
							glm::cross(joint->transform.quaternion, p1quat);
					}
					else {
						//For all other children joints
						//Update its translation
						joint->transform.translation = joint->parent->transform.translation
							+ (joint->parent->transform.quaternion * rt.translation);

						//inherit the parent's orientation
						joint->transform.quaternion =
							glm::cross(joint->parent->transform.quaternion, rt.quaternion);
					}
				}

				//Add all children joints to the queue
				for (JOINT* jointChild : joint->children) {
					jointQueue.push(jointChild);
				}
			}

			//Step 3: Update new locations of points, and check if solution is good
			p0 = lshldr->transform.translation;
			p1 = larm->transform.translation;
			p2 = lforearm->transform.translation;
			p3 = lhand->transform.translation;

			//If error is trivial enough, we assume it is "good enough"
			if (p3.x - x < errorMargin &&
				p3.y - y < errorMargin &&
				p3.x - x < errorMargin) {
				isNotGoodEnough = false;
			}
		}
	}





    // ----------------------------------------------------------
    // Do not touch
    // ----------------------------------------------------------
    clock_t end_time = clock();
    float elapsed = (end_time - start_time) / (float)CLOCKS_PER_SEC;
    cout << "Solving done in " << elapsed * 1000 << " ms." << endl;
}
