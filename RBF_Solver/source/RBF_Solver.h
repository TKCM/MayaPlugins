#pragma once
// MayaAPI
#include <maya/MPxNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MGlobal.h>

#include "RBF_Solver_functions.h"

class RBFSolver : public MPxNode{
	public:
		RBFSolver () {};
		virtual ~RBFSolver () {};

		virtual MStatus compute(const MPlug &plug, MDataBlock& dataBlock) override;
		static  void*   creator() { return new RBFSolver ();  };
		static  MStatus initialize();
		MStatus setDependentsDirty ( const MPlug& plug, MPlugArray& plugArray );

		static MObject type;
		static MObject referenceList;
		static MObject reference;
		static MObject valuesList;
		static MObject values;
		static MObject input;
		static MObject result;

		RBF rbf;
		bool setupDirty = true;
};