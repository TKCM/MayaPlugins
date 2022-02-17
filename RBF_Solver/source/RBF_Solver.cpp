#include "RBF_Solver.h"

#define kPluginNodeId 0x19970002
#define kPluginNodeName "RBF_Solver"

MObject RBFSolver::type;
MObject RBFSolver::reference;
MObject RBFSolver::referenceList;
MObject RBFSolver::values;
MObject RBFSolver::valuesList;
MObject RBFSolver::input;
MObject RBFSolver::result;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MStatus RBFSolver::compute ( const MPlug &plug, MDataBlock& dataBlock )
{
	MStatus status;
	///////////////////////////////////////////////////////////////
	// input

	MArrayDataHandle inputHandle = dataBlock.inputArrayValue ( input );
	std::vector<float> _input ( inputHandle.elementCount () );
	for (uInt i = 0; i < inputHandle.elementCount (); ++i) {
		inputHandle.jumpToElement ( i );
		_input[i] = inputHandle.inputValue ().asFloat ();
	}
	Eigen::VectorXf p = Eigen::VectorXf::Map ( &_input[0], _input.size () );

	if (this->setupDirty) {
		int _type = dataBlock.inputValue ( type ).asInt ();

		MArrayDataHandle rListHandle = dataBlock.inputArrayValue ( referenceList );
		std::vector< std::vector<float>> _reference ( rListHandle.elementCount () );
		size_t rElementSize = 10000;
		for (uInt i = 0; i < rListHandle.elementCount (); ++i) {
			MStatus s = rListHandle.jumpToElement ( i );
			if (s != MStatus::kSuccess) { continue; }

			MDataHandle rHandle = rListHandle.inputValue ().child ( reference );
			MArrayDataHandle rDataHandle ( rHandle );

			int maxCount = rDataHandle.elementCount ();
			int j = 0, c = 0;
			while (c < maxCount) {
				if (rDataHandle.jumpToElement ( j ) == MStatus::kSuccess) {
					_reference[i].push_back ( rDataHandle.inputValue ().asFloat () );
					++c;
				}
				++j;
			}
			rElementSize = std::min ( rElementSize, _reference[i].size () );
		}
		for (int i = 1; i < _reference.size (); ++i) {
			_reference[i].resize ( rElementSize );
		}

		MArrayDataHandle vListHandle = dataBlock.inputArrayValue ( valuesList );
		std::vector < std::vector<float>> _values ( vListHandle.elementCount () );
		size_t vElementSize = 10000;
		for (uInt i = 0; i < vListHandle.elementCount (); ++i) {
			MStatus s = vListHandle.jumpToElement ( i );
			if (s != MStatus::kSuccess) { continue; }

			MDataHandle vHandle = vListHandle.inputValue ().child ( values );
			MArrayDataHandle vDataHandle ( vHandle );

			int maxCount = vDataHandle.elementCount ();
			int j = 0, c = 0;
			while (c < maxCount) {
				if (vDataHandle.jumpToElement ( j ) == MStatus::kSuccess) {
					_values[i].push_back ( vDataHandle.inputValue ().asFloat () );
					++c;
				}
				++j;
			}
			vElementSize = std::min ( vElementSize, _values[i].size () );
		}
		for (int i = 1; i < _reference.size (); ++i) {
			_values[i].resize ( vElementSize );
		}

		///////////////////////////////////////////////////////////////
		// core
		Eigen::MatrixXf targetMat = RBFSolverFn::array2DToMat ( _reference );
		Eigen::MatrixXf retargetMat = RBFSolverFn::array2DToMat ( _values );
		
		// RBF setup
		if (RBFSolverFn::setup ( this->rbf, targetMat, retargetMat, _type ) == false) {
			return status;
		}

		this->setupDirty = false;
	}

	// RBF solve
	Eigen::VectorXf _result;
	if (RBFSolverFn::solve ( _result, this->rbf, p ) == false) {
		return status;
	}
	
	///////////////////////////////////////////////////////////////
	// output
	MArrayDataHandle resultHandle = dataBlock.outputArrayValue ( result );
	MArrayDataBuilder resultBuilder = resultHandle.builder ();
	for (int i = 0; i< int ( _result.size () ); ++i) {
		resultBuilder.addElement ( i ).set ( _result[i] );
		resultBuilder.addElement ( i ).setClean ();
	}
	resultHandle.set ( resultBuilder );
	resultHandle.setAllClean ();

	dataBlock.setClean(plug);
	return status;
}

MStatus RBFSolver::setDependentsDirty ( const MPlug& plug, MPlugArray& plugArray ) {
	if (plug == this->type || plug == this->valuesList || plug == this->referenceList || plug == this->values || plug == this->reference) {
		this->setupDirty = true;
	}
	return MPxNode::setDependentsDirty ( plug, plugArray );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MStatus RBFSolver::initialize()
{
	MFnTypedAttribute gAttr;
	MFnNumericAttribute nAttr;
	MFnCompoundAttribute cAttr;

	type = nAttr.create("setupType", "setupType", MFnNumericData::kInt);
	nAttr.setKeyable(true);
	nAttr.setDefault ( 3 );
	input = nAttr.create ( "input", "input", MFnNumericData::kFloat );
	nAttr.setArray ( true );
	nAttr.setKeyable ( true );
	result = nAttr.create ( "result", "result", MFnNumericData::kFloat );
	nAttr.setArray ( true );
	nAttr.setReadable ( true );
	nAttr.setUsesArrayDataBuilder ( true );

	reference = nAttr.create ( "reference", "reference", MFnNumericData::kFloat );
	nAttr.setArray ( true );
	nAttr.setKeyable ( true );
	values = nAttr.create ( "values", "values", MFnNumericData::kFloat );
	nAttr.setArray ( true );
	nAttr.setKeyable ( true );

	// build 2D array port
	referenceList = cAttr.create ( "setupReferenceList", "setupReferenceList" );
	cAttr.setArray ( true );
	cAttr.addChild ( reference );
	valuesList = cAttr.create ( "setupValuesList", "setupValuesList" );
	cAttr.setArray ( true );
	cAttr.addChild ( values );

	addAttribute ( type );
	addAttribute ( referenceList );
	addAttribute ( reference );
	addAttribute ( valuesList );
	addAttribute ( values );
	addAttribute ( input );
	addAttribute ( result );

	attributeAffects( type, result );
	attributeAffects( reference, result );
	attributeAffects( values, result );
	attributeAffects ( input, result );

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "TKCM", "20220216", "Any");

	status = plugin.registerNode(kPluginNodeName, kPluginNodeId, RBFSolver::creator, RBFSolver::initialize);
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