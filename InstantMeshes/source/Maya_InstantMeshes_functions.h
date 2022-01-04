#pragma once
// MayaAPI
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MVector.h>
#include <maya/MMatrix.h>

// InstantMeshes
#include <src/batch.h>
#include <src/meshio.h>
#include <src/dedge.h>
#include <src/subdivide.h>
#include <src/meshstats.h>
#include <src/hierarchy.h> 
#include <src/field.h>
#include <src/normal.h>
#include <src/extract.h>
#include <src/bvh.h>
#include <src/smoothcurve.h>
#include <src/extract.h>

// Eigen
#include <Eigen/Dense>

namespace MInstantMeshesFn{
	void ApplayTransform( MFnMesh& mesh, const MMatrix& mat );

	void MFnMeshToInstantMeshesTopology( MatrixXu& F, MatrixXf& N, MatrixXf& V, const MFnMesh& mayaMesh );
	MObject InstantMeshesTopologyToMFnMesh( MFnMesh& mayaMesh, const MatrixXu& F, const MatrixXf& N, const MatrixXf& V );
}