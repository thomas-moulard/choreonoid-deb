/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Vector3Seq.h"
#include "PlainSeqFormatLoader.h"
#include "YamlNodes.h"
#include "YamlWriter.h"
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;


Vector3Seq::Vector3Seq(int nFrames, double frameRate)
    : BaseSeqType("Vector3Seq", nFrames, frameRate)
{

}


Vector3Seq::Vector3Seq(const Vector3Seq& org)
    : BaseSeqType(org)
{

}


Vector3Seq::~Vector3Seq()
{

}


bool Vector3Seq::loadPlainFormat(const std::string& filename)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        setIoErrorMessage(loader.errorMessage());
        return false;
    }

    if(loader.numParts() < 3){
        setIoErrorMessage(filename + "does not have 3 columns for 3d vector elements");
        return false;
    }
  
    setNumFrames(loader.numFrames());
    setFrameRate(1.0 / loader.timeStep());

    int frame = 0;
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        vector<double>& data = *it;
        (*this)[frame++] << data[1], data[2], data[3];
    }

    return true;
}


bool Vector3Seq::saveAsPlainFormat(const std::string& filename)
{
    ofstream os(filename.c_str());
    os.setf(ios::fixed);

    if(!os){
        setIoErrorMessage(filename + " cannot be opened.");
        return false;
    }

    static format f("%1$.4f %2$.6f %3$.6f %4$.6f\n");

    const int n = numFrames();
    const double r = frameRate();

    for(int i=0; i < n; ++i){
        const Vector3& v = (*this)[i];
        os << (f % (i / r) % v.x() % v.y() % v.z());
    }
    
    return true;
}


bool Vector3Seq::write(YamlWriter& writer)
{
    bool result = false;
    
    writer.startMapping();

    if(BaseSeqType::write(writer)){

        writer.putKey("frames");
        writer.startSequence();
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            writer.startFlowStyleSequence();
            const Vector3& v = (*this)[i];
            for(int j=0; j < 3; ++j){
                writer.putScalar(v[j]);
            }
            writer.endSequence();
        }
        writer.endSequence();
        
        result = true;
    }

    writer.endMapping();

    return result;
}


bool Vector3Seq::read(const YamlMapping& archive)
{
    if(BaseSeqType::read(archive) && (archive["type"].toString() == seqType())){
        const YamlSequence& frames = *archive.findSequence("frames");
        if(frames.isValid()){
            const int n = frames.size();
            setNumFrames(n);
            for(int i=0; i < n; ++i){
                const YamlSequence& frame = *frames[i].toSequence();
                Vector3& v = (*this)[i];
                for(int j=0; j < 3; ++j){
                    v[j] = frame[j].toDouble();
                }
            }
        }
        return true;
    }
    
    return false;
}
