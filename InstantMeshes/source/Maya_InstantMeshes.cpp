#include "Maya_InstantMeshes.h"

// ノードID
#define kPluginNodeId 0x19970000
// ノード名
#define kPluginNodeName "Maya_InstantMeshes"

// メンバ変数
MObject Maya_InstantMeshes::sourceMesh;
MObject Maya_InstantMeshes::sourceMeshTransform;
MObject Maya_InstantMeshes::resultMesh;
MObject Maya_InstantMeshes::remeshAs;
MObject Maya_InstantMeshes::vertexCount;
MObject Maya_InstantMeshes::targetType;
MObject Maya_InstantMeshes::deterministic;
MObject Maya_InstantMeshes::scaleVal;
MObject Maya_InstantMeshes::faceCount;
MObject Maya_InstantMeshes::creaseAngle;
MObject Maya_InstantMeshes::extrinsic;
MObject Maya_InstantMeshes::alignToBoundaries;
MObject Maya_InstantMeshes::smoothIter;
MObject Maya_InstantMeshes::pureQuad;

MStatus Maya_InstantMeshes::compute ( const MPlug &plug, MDataBlock& dataBlock ) 
{
	// input
	MFnMesh inSourceMesh = dataBlock.inputValue ( sourceMesh ).asMesh ();
	MMatrix inSourceMatrix = dataBlock.inputValue ( sourceMeshTransform ).asMatrix ();

	if (inSourceMesh.numPolygons () < 4) { return MStatus::kFailure; }

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// core

	// viewer.h:72
	Float scale = -1;
	int face_count = -1;
	int vertex_count = -1;
	int knn_points = 10;
	Float creaseAngle = -1;

	// viewer.cpp:57
	int rosy, posy;
	switch (dataBlock.inputValue ( remeshAs ).asInt ()) {
	case 0:
		rosy = 6;
		posy = 3;
		break;
	case 1:
		rosy = 2;
		posy = 4;
		break;
	case 2:
		rosy = 4;
		posy = 4;
		break;
	default:
		rosy = 4;
		posy = 4;
		break;
	}

	switch (dataBlock.inputValue ( targetType ).asInt ()) {
	case 0:
		vertex_count = dataBlock.inputValue ( vertexCount ).asInt ();
		break;
	case 1:
		face_count = dataBlock.inputValue ( faceCount ).asInt ();
		break;
	case 2:
		scale = dataBlock.inputValue ( scaleVal ).asFloat ();
		break;
	default:
		vertex_count = dataBlock.inputValue ( vertexCount ).asInt ();
		break;
	}

	MFnMesh instantMesh;
	MObject outFnMeshObj;
	// batch.cpp:49
	try {
		MatrixXu F;
		MatrixXf N;
		MatrixXf V;
		VectorXf A;
		std::set<uint32_t> crease_in, crease_out;
		BVH* bvh = nullptr;
		AdjacencyMatrix adj = nullptr;

		// Burgess.PolyMeshTopo to InstantMeshesMatrixes
		if (this->dirty == true) {
			// apply mesh transform
			MInstantMeshesFn::ApplayTransform ( inSourceMesh, inSourceMatrix );
			MInstantMeshesFn::MFnMeshToInstantMeshesTopology ( this->mF, this->mN, this->mV, inSourceMesh );
			this->dirty = false;
		}
		F = this->mF;
		N = this->mN;
		V = this->mV;

		bool pointcloud = F.size () == 0;
		MeshStats stats = compute_mesh_stats ( F, V, dataBlock.inputValue ( deterministic ).asBool () );
		if (pointcloud) {
			bvh = new BVH ( &F, &V, &N, stats.mAABB );
			bvh->build ();
			adj = generate_adjacency_matrix_pointcloud ( V, N, bvh, stats, knn_points, dataBlock.inputValue ( deterministic ).asBool () );
			A.resize ( V.cols () );
			A.setConstant ( 1.0f );
		}

		if (scale < 0 && vertex_count < 0 && face_count < 0) {
			vertex_count = V.cols () / 16;
		}

		if (scale > 0) {
			Float face_area = posy == 4 ? (scale * scale) : (std::sqrt ( 3.f ) / 4.f * scale * scale);
			face_count = stats.mSurfaceArea / face_area;
			vertex_count = posy == 4 ? face_count : (face_count / 2);
		}
		else if (face_count > 0) {
			Float face_area = stats.mSurfaceArea / face_count;
			vertex_count = posy == 4 ? face_count : (face_count / 2);
			scale = posy == 4 ? std::sqrt ( face_area ) : (2 * std::sqrt ( face_area * std::sqrt ( 1.f / 3.f ) ));
		}
		else if (vertex_count > 0) {
			face_count = posy == 4 ? vertex_count : (vertex_count * 2);
			Float face_area = stats.mSurfaceArea / face_count;
			scale = posy == 4 ? std::sqrt ( face_area ) : (2 * std::sqrt ( face_area * std::sqrt ( 1.f / 3.f ) ));
		}

		MultiResolutionHierarchy mRes;

		if (!pointcloud) {
			// Subdivide the mesh if necessary
			VectorXu V2E, E2E;
			VectorXb boundary, nonManifold;
			if (stats.mMaximumEdgeLength * 2 > scale || stats.mMaximumEdgeLength > stats.mAverageEdgeLength * 2) {
				build_dedge ( F, V, V2E, E2E, boundary, nonManifold );
				subdivide ( F, V, V2E, E2E, boundary, nonManifold, std::min ( scale / 2, (Float)stats.mAverageEdgeLength * 2 ), dataBlock.inputValue ( deterministic ).asBool () );
			}

			// Compute a directed edge data structure
			build_dedge ( F, V, V2E, E2E, boundary, nonManifold );

			// Compute adjacency matrix
			adj = generate_adjacency_matrix_uniform ( F, V2E, E2E, nonManifold );

			// Compute vertex/crease normals
			if (creaseAngle >= 0)
				generate_crease_normals ( F, V, V2E, E2E, boundary, nonManifold, creaseAngle, N, crease_in );
			else
				generate_smooth_normals ( F, V, V2E, E2E, nonManifold, N );

			// Compute dual vertex areas
			compute_dual_vertex_areas ( F, V, V2E, E2E, nonManifold, A );

			mRes.setE2E ( std::move ( E2E ) );
		}

		// Build multi-resolution hierarrchy
		mRes.setAdj ( std::move ( adj ) );
		mRes.setF ( std::move ( F ) );
		mRes.setV ( std::move ( V ) );
		mRes.setA ( std::move ( A ) );
		mRes.setN ( std::move ( N ) );
		mRes.setScale ( scale );
		mRes.build ( dataBlock.inputValue ( deterministic ).asBool () );
		mRes.resetSolution ();

		if (dataBlock.inputValue ( alignToBoundaries ).asBool () && !pointcloud) {
			mRes.clearConstraints ();
			for (uint32_t i = 0; i < 3 * mRes.F ().cols (); ++i) {
				if (mRes.E2E ()[i] == INVALID) {
					uint32_t i0 = mRes.F ()(i % 3, i / 3);
					uint32_t i1 = mRes.F ()((i + 1) % 3, i / 3);
					Vector3f p0 = mRes.V ().col ( i0 ), p1 = mRes.V ().col ( i1 );
					Vector3f edge = p1 - p0;
					if (edge.squaredNorm () > 0) {
						edge.normalize ();
						mRes.CO ().col ( i0 ) = p0;
						mRes.CO ().col ( i1 ) = p1;
						mRes.CQ ().col ( i0 ) = mRes.CQ ().col ( i1 ) = edge;
						mRes.CQw ()[i0] = mRes.CQw ()[i1] = mRes.COw ()[i0] =
							mRes.COw ()[i1] = 1.0f;
					}
				}
			}
			mRes.propagateConstraints ( rosy, posy );
		}

		bvh = new BVH ( &mRes.F (), &mRes.V (), &mRes.N (), stats.mAABB );
		bvh->build ();

		// batch.cpp:170
		Optimizer optimizer ( mRes, false );
		optimizer.setRoSy ( rosy );
		optimizer.setPoSy ( posy );
		optimizer.setExtrinsic ( dataBlock.inputValue ( extrinsic ).asBool () );

		optimizer.optimizeOrientations ( -1 );
		optimizer.notify ();
		optimizer.wait ();
		std::map<uint32_t, uint32_t> sing;
		compute_orientation_singularities ( mRes, sing, dataBlock.inputValue ( extrinsic ).asBool (), rosy );

		optimizer.optimizePositions ( -1 );
		optimizer.notify ();
		optimizer.wait ();

		optimizer.shutdown ();

		MatrixXf O_extr, N_extr, Nf_extr;
		std::vector<std::vector<TaggedLink>> adj_extr;
		extract_graph ( optimizer.mRes, dataBlock.inputValue ( extrinsic ).asBool (), optimizer.rosy (), optimizer.posy (), adj_extr, O_extr, N_extr,
			crease_in, crease_out, dataBlock.inputValue ( deterministic ).asBool () );

		MatrixXu F_extr;
		extract_faces ( adj_extr, O_extr, N_extr, Nf_extr, F_extr, posy,
			optimizer.mRes.scale (), crease_out, true, dataBlock.inputValue ( pureQuad ).asBool (), bvh, dataBlock.inputValue ( smoothIter ).asInt () );

		outFnMeshObj = MInstantMeshesFn::InstantMeshesTopologyToMFnMesh ( instantMesh, F_extr, Nf_extr, O_extr );

		if (bvh) {
			delete bvh;
		}
	}
	catch (const std::exception& e) {
		return MStatus::kFailure;
	}

	// output
	if (0 < instantMesh.numPolygons ()) {
		MDataHandle outFnMeshHandle = dataBlock.outputValue ( resultMesh );
		outFnMeshHandle.setMObject ( outFnMeshObj );
	}

	dataBlock.setClean ( plug );
	return MStatus::kSuccess;
}

MStatus Maya_InstantMeshes::setDependentsDirty ( const MPlug& plug, MPlugArray& plugArray ) {
	if ( plug == this->sourceMesh || plug == this->sourceMeshTransform ) 
	{
		this->dirty = true;
	}

	return MStatus::kSuccess;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MStatus Maya_InstantMeshes::initialize ()
{
	MFnTypedAttribute tAttr;
	MFnNumericAttribute nAttr;
	MFnMatrixAttribute mAttr;

	sourceMesh = tAttr.create ( "sourceMesh", "sourceMesh", MFnData::kMesh );
	tAttr.setKeyable ( true );
	sourceMeshTransform = mAttr.create ( "sourceMeshTransform", "sourceMeshTransform" );
	mAttr.setKeyable ( true );
	resultMesh = tAttr.create ( "resultMesh", "resultMesh", MFnData::kMesh );
	tAttr.setWritable ( true );
	remeshAs = nAttr.create ( "remeshAs", "remeshAs", MFnNumericData::kInt );
	nAttr.setKeyable ( true );
	nAttr.setMin ( 0 );
	nAttr.setMax ( 2 );
	nAttr.setDefault ( 2 );
	vertexCount = nAttr.create ( "vertexCount", "vertexCount", MFnNumericData::kInt );
	nAttr.setKeyable ( true );
	nAttr.setMin ( 10 );
	nAttr.setMax ( 50000 );
	nAttr.setDefault ( 500 );
	targetType = nAttr.create ( "targetType", "targetType", MFnNumericData::kInt );
	nAttr.setKeyable ( true );
	nAttr.setMin ( 0 );
	nAttr.setMax ( 2 );
	nAttr.setDefault ( 0 );
	deterministic = nAttr.create ( "deterministic", "deterministic", MFnNumericData::kBoolean );
	nAttr.setKeyable ( true );
	nAttr.setDefault ( true );
	scaleVal = nAttr.create ( "scale", "scale", MFnNumericData::kFloat );
	nAttr.setKeyable ( true );
	nAttr.setMin ( 10.0 );
	nAttr.setMax ( 300.0 );
	nAttr.setDefault ( 200.0 );
	faceCount = nAttr.create ( "faceCount", "faceCount", MFnNumericData::kInt );
	nAttr.setKeyable ( true );
	nAttr.setMin ( 10 );
	nAttr.setMax ( 30000 );
	nAttr.setDefault ( 300 );
	creaseAngle = nAttr.create ( "creaseAngle", "creaseAnglea", MFnNumericData::kFloat );
	nAttr.setKeyable ( true );
	extrinsic = nAttr.create ( "extrinsic", "extrinsic", MFnNumericData::kBoolean );
	nAttr.setKeyable ( true );
	nAttr.setDefault ( true );
	alignToBoundaries = nAttr.create ( "alignToBoundaries", "alignToBoundaries", MFnNumericData::kBoolean );
	nAttr.setKeyable ( true );
	nAttr.setDefault ( true );
	smoothIter = nAttr.create ( "smoothIter", "smoothIter", MFnNumericData::kInt );
	nAttr.setKeyable ( true );
	nAttr.setMin ( 0 );
	nAttr.setMax ( 10 );
	nAttr.setDefault ( 0 );
	pureQuad = nAttr.create ( "pureQuad", "pureQuad", MFnNumericData::kBoolean );
	nAttr.setKeyable ( true );

	addAttribute ( sourceMesh );
	addAttribute ( sourceMeshTransform );
	addAttribute ( resultMesh );
	addAttribute ( remeshAs );
	addAttribute ( targetType );
	addAttribute ( vertexCount );
	addAttribute ( faceCount );
	addAttribute ( scaleVal );
	addAttribute ( pureQuad );
	addAttribute ( smoothIter );
	addAttribute ( deterministic );
	addAttribute ( extrinsic );
	addAttribute ( alignToBoundaries );
	addAttribute ( creaseAngle );

	attributeAffects ( sourceMesh, resultMesh );
	attributeAffects ( sourceMeshTransform, resultMesh );
	attributeAffects ( vertexCount, resultMesh );
	attributeAffects ( remeshAs, resultMesh );
	attributeAffects ( targetType, resultMesh );
	attributeAffects ( deterministic, resultMesh );
	attributeAffects ( scaleVal, resultMesh );
	attributeAffects ( faceCount, resultMesh );
	attributeAffects ( creaseAngle, resultMesh );
	attributeAffects ( extrinsic, resultMesh );
	attributeAffects ( alignToBoundaries, resultMesh );
	attributeAffects ( smoothIter, resultMesh );
	attributeAffects ( pureQuad, resultMesh );

	return MS::kSuccess;
}

MStatus initializePlugin ( MObject obj )
{
	MStatus   status;
	MFnPlugin plugin ( obj, "TKCM", "20211231", "Any" );

	status = plugin.registerNode ( kPluginNodeName, kPluginNodeId, Maya_InstantMeshes::creator, Maya_InstantMeshes::initialize );
	CHECK_MSTATUS_AND_RETURN_IT ( status );

	return status;
}

MStatus uninitializePlugin ( MObject obj )
{
	MStatus   status;
	MFnPlugin plugin ( obj );

	status = plugin.deregisterNode ( kPluginNodeId );
	CHECK_MSTATUS_AND_RETURN_IT ( status );

	return status;
}