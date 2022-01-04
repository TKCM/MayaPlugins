#include "Maya_InstantMeshes_functions.h"

namespace MInstantMeshesFn{
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void ApplayTransform( MFnMesh& mesh, const MMatrix& mat ){
		MPointArray pts;
		mesh.getPoints(pts);
		#pragma omp parallel for 
		for ( int ptID = 0; ptID < pts.length(); ++ptID ){
			pts[ptID] = pts[ptID] * mat;
		}
		mesh.setPoints( pts );

		MFloatVectorArray nmls;
		mesh.getNormals( nmls );
		#pragma omp parallel for 
		for ( int nmlID = 0; nmlID < nmls.length(); ++nmlID ){
			nmls[nmlID] = MVector{ nmls[nmlID] } * mat;
		}
		mesh.setNormals( nmls );
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void MFnMeshToInstantMeshesTopology( MatrixXu& F, MatrixXf& N, MatrixXf& V, const MFnMesh& mayaMesh ){
		// packed polygon point indices
		std::vector<unsigned int> ids;
		ids.reserve( 3 * mayaMesh.numPolygons() );
		for ( int polyID = 0; polyID < mayaMesh.numPolygons(); ++polyID ){
			MIntArray polyPointIDs;
			mayaMesh.getPolygonVertices( polyID, polyPointIDs );

			int trianglePoyCount = polyPointIDs.length() - 2;
			for ( int i = 0; i < trianglePoyCount; ++i ){
				ids.push_back( polyPointIDs[0] );
				ids.push_back( polyPointIDs[i + 1] );
				ids.push_back( polyPointIDs[i + 2] );
			}
		}
		F = Eigen::Map<MatrixXu> ( ids.data(), 3, ids.size() / 3 );

		// point position
		MPointArray pointPositions;
		mayaMesh.getPoints( pointPositions );
		V.resize( 3, pointPositions.length() );
		#pragma omp parallel for if ( 300<pointPositions.length() )
		for ( int i = 0; i < pointPositions.length(); ++i ){
			V( 0, i ) = pointPositions[i].x;
			V( 1, i ) = pointPositions[i].y;
			V( 2, i ) = pointPositions[i].z;
		}

		// point normal
		N.resize( 3, pointPositions.length() );
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	MObject InstantMeshesTopologyToMFnMesh( MFnMesh& mayaMesh, const MatrixXu& F, const MatrixXf& N, const MatrixXf& V ){
		MFnMeshData outFnMeshData;
		MObject result = outFnMeshData.create();
		
		// polygon info
		MIntArray polygonPackedPointIDs, polygonSizeList;
		for ( uint32_t f = 0; f < F.cols(); ++f ){
			if ( F.rows() == 4 && F( 2, f ) == F( 3, f ) ){ continue; }

			int nbF = 0;
			for ( uint32_t i = 0; i < F.rows(); ++i ){
				polygonPackedPointIDs.append( F( i, f ) );
				nbF++;
			}
			if ( 3 <= nbF ){
				polygonSizeList.append( nbF );
			}
		}

		// point position
		MPointArray pointPositions( V.cols() );
		#pragma omp parallel for if ( 300<V.cols() )
		for ( int i = 0; i < V.cols(); ++i ){
			pointPositions[i].x = V( 0, i );
			pointPositions[i].y = V( 1, i );
			pointPositions[i].z = V( 2, i );
		}

		std::map<uint32_t, std::pair<uint32_t, std::map<uint32_t, uint32_t>>> irregular;
		size_t nIrregular = 0;
		if ( F.rows() == 4 ){
			for ( uint32_t f = 0; f < F.cols(); ++f ){
				if ( F( 2, f ) == F( 3, f ) ){
					nIrregular++;
					auto& value = irregular[F( 2, f )];
					value.first = f;
					value.second[F( 0, f )] = F( 1, f );
				}
			}
		}
		for ( auto item : irregular ){
			auto face = item.second;
			uint32_t v = face.second.begin()->first, first = v, i = 0;
			int nbF = 0;
			while ( true ){
				polygonPackedPointIDs.append( v );
				nbF++;
				v = face.second[v];
				i++;

				if ( v == first || i == face.second.size() ){ break; }
			}
			if ( 3 <= nbF ){
				polygonSizeList.append( nbF );
			}
			nbF = 0;
			while ( i != face.second.size() ){
				polygonPackedPointIDs.append( v );
				nbF++;
				i++;
			}
			if ( 3 <= nbF ){
				polygonSizeList.append( nbF );
			}
		}

		// create MFnMesh
		mayaMesh.create( pointPositions.length(), polygonSizeList.length(), pointPositions, polygonSizeList, polygonPackedPointIDs, result );
		mayaMesh.updateSurface();

		return result;
	}
}