
/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_WRITER_INCLUDED
#define CNOID_UTIL_VRML_WRITER_INCLUDED

#include "VrmlNodes.h"
#include <map>
#include <string>
#include <ostream>
#include "exportdecl.h"

namespace cnoid {

    class VrmlWriter;

    class CNOID_EXPORT VrmlWriter
    {
    public:
	VrmlWriter(std::ostream& out);
  
	void writeHeader();
	bool writeNode(VrmlNodePtr node);

	struct TIndent {
	    void clear() { n = 0; spaces.resize(n); }
	    inline TIndent& operator++() { n += 2; spaces.resize(n, ' '); return *this; }
	    inline TIndent& operator--() { 
		n -= 2;
		if(n < 0) { n = 0; }
		spaces.resize(n, ' '); return *this; 
	    }
	    std::string spaces;
	    int n;
	};

    private:
	std::ostream& out;

	TIndent indent;

	void registerNodeMethodMap();
	template <class MFValues> void writeMFValues(MFValues values, int numColumn);
	void writeMFInt32SeparatedByMinusValue(MFInt32& values);
	void writeNodeIter(VrmlNodePtr node);
	void beginNode(const char* nodename, VrmlNodePtr node);
	void endNode();
	void writeGroupNode(VrmlNodePtr node);
	void writeGroupFields(VrmlGroupPtr group);
	void writeTransformNode(VrmlNodePtr node);
	void writeShapeNode(VrmlNodePtr node);
	void writeAppearanceNode(VrmlAppearancePtr appearance);
	void writeMaterialNode(VrmlMaterialPtr material);
	void writeIndexedFaceSetNode(VrmlNodePtr node);
	void writeCoordinateNode(VrmlCoordinatePtr coord);

    };

};


#endif
