#include "RBF_MeshRetargeter.h"

#define kPluginNodeId 0x19970001
#define kPluginNodeName "RBFMeshRetargeter"

MObject RBFMeshRetargeter::targetMesh;
MObject RBFMeshRetargeter::retargetMesh;
MObject RBFMeshRetargeter::origMesh;
MObject RBFMeshRetargeter::outMesh;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MStatus RBFMeshRetargeter::compute(const MPlug &plug, MDataBlock& dataBlock) 
{
	MStatus status;
	///////////////////////////////////////////////////////////////
	// input
	MObject origObj = dataBlock.inputValue(origMesh).asMesh();
	MObject targetObj = dataBlock.inputValue(targetMesh).asMesh();
	MObject retargetObj = dataBlock.inputValue(retargetMesh).asMesh();
	MDataHandle outMeshHandle = dataBlock.outputValue(outMesh);
	if (origObj.isNull() || targetObj.isNull() || retargetObj.isNull()) {
		return status;
	}

	MFnMesh origFnMesh(origObj);
	MFnMesh targetFnMesh(targetObj);
	MFnMesh retargetFnMesh(retargetObj);
	if (targetFnMesh.numVertices() != retargetFnMesh.numVertices()) {
		return status;
	}

	MObject meshObject = MFnMeshData().create();
	MFnMesh	resultMesh;
	resultMesh.copy(origObj, meshObject);	

	///////////////////////////////////////////////////////////////
	// core
	Eigen::MatrixXf targetPoiPosMat ( targetFnMesh.numVertices (), 3 );
	#pragma omp parallel for
	for (int i = 0; i < targetFnMesh.numVertices(); i++) {
		MPoint poiPos;
		targetFnMesh.getPoint(i, poiPos);
		targetPoiPosMat ( i, 0 ) = poiPos.x;
		targetPoiPosMat ( i, 1 ) = poiPos.y;
		targetPoiPosMat ( i, 2 ) = poiPos.z;
	}

	Eigen::MatrixXf retargetPoiPosMat ( retargetFnMesh.numVertices (), 3 );
	#pragma omp parallel for
	for (int i = 0; i < retargetFnMesh.numVertices(); i++) {
		MPoint poiPos;
		retargetFnMesh.getPoint(i, poiPos);
		retargetPoiPosMat ( i, 0 ) = poiPos.x;
		retargetPoiPosMat ( i, 1 ) = poiPos.y;
		retargetPoiPosMat ( i, 2 ) = poiPos.z;
	}

	// RBF setup
	RBF rbf;
	RBFRetargeterFn::setup ( rbf, targetPoiPosMat, retargetPoiPosMat, 3 ); // rbf type = Gaussian

	// RBF solve
	#pragma omp parallel for
	for (int i = 0; i < origFnMesh.numVertices (); i++) {
		// get orig point position
		MPoint poiPos;
		origFnMesh.getPoint ( i, poiPos );

		// create vector
		Eigen::VectorXf p (3);
		p ( 0 ) = poiPos.x;
		p ( 1 ) = poiPos.y;
		p ( 2 ) = poiPos.z;

		// solve
		Eigen::VectorXf result;
		RBFRetargeterFn::solve ( result, rbf, p );

		// set point position
		poiPos = MPoint{ result ( 0 ), result ( 1 ), result ( 2 ) };
		resultMesh.setPoint(i, poiPos );
	}

	///////////////////////////////////////////////////////////////
	// output
	outMeshHandle.setMObject(meshObject);
	
	dataBlock.setClean(plug);
	return status;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MStatus RBFMeshRetargeter::initialize()
{
	MFnTypedAttribute gAttr;

	targetMesh = gAttr.create("targetMesh", "targetMesh", MFnData::kMesh);
	gAttr.setReadable(false);
	gAttr.setKeyable(true);
	retargetMesh = gAttr.create("retargetMesh", "retargetMesh", MFnData::kMesh);
	gAttr.setReadable(false);
	gAttr.setKeyable(true);
	origMesh = gAttr.create("inputMesh", "inputMesh", MFnData::kMesh);
	gAttr.setReadable(false);
	gAttr.setKeyable(true);
	outMesh = gAttr.create("outputMesh", "outputMesh", MFnData::kMesh);
	gAttr.setWritable(false);

	addAttribute(targetMesh);
	addAttribute(retargetMesh);
	addAttribute(origMesh);
	addAttribute(outMesh);

	attributeAffects(targetMesh, outMesh);
	attributeAffects(retargetMesh, outMesh);
	attributeAffects(origMesh, outMesh);

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "TKCM", "20220106", "Any");

	status = plugin.registerNode(kPluginNodeName, kPluginNodeId, RBFMeshRetargeter::creator, RBFMeshRetargeter::initialize);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(kPluginNodeId);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;
}