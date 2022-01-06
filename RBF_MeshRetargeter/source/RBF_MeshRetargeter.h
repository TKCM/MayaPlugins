#pragma once

// MayaAPI
#include <maya/MPxNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MGlobal.h>

#include "RBF_MeshRetargeter_functions.h"

class RBFMeshRetargeter : public MPxNode{
	public:
		RBFMeshRetargeter () {};
		virtual ~RBFMeshRetargeter () {};

		virtual MStatus compute(const MPlug &plug, MDataBlock& dataBlock) override;
		static  void*   creator() { return new RBFMeshRetargeter ();  };
		static  MStatus initialize();

		static MObject targetMesh;
		static MObject retargetMesh;
		static MObject origMesh;
		static MObject outMesh;
};