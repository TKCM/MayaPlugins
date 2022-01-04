#pragma once
// MayaAPI
#include <maya/MPxNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MTypes.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
 
#include "Maya_InstantMeshes_functions.h"

class Maya_InstantMeshes : public MPxNode {
public:
	Maya_InstantMeshes () {};
	virtual ~Maya_InstantMeshes () {};

	virtual MStatus compute ( const MPlug &plug, MDataBlock& dataBlock ) override;
	static  void*   creator () { return new Maya_InstantMeshes (); };
	virtual MStatus setDependentsDirty ( const MPlug& plug, MPlugArray& plugArray );
	static  MStatus initialize ();

	static MObject sourceMesh;
	static MObject sourceMeshTransform;
	static MObject resultMesh;
	static MObject remeshAs;
	static MObject vertexCount;
	static MObject targetType;
	static MObject deterministic;
	static MObject scaleVal;
	static MObject faceCount;
	static MObject creaseAngle;
	static MObject extrinsic;
	static MObject alignToBoundaries;
	static MObject smoothIter;
	static MObject pureQuad;

	// InstantMeshes topology
	bool dirty = true;
	MatrixXu mF;
	MatrixXf mN;
	MatrixXf mV;
};