/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeq.h"
#include "PlainSeqFormatLoader.h"
#include "YamlNodes.h"
#include "YamlWriter.h"

using namespace std;
using namespace cnoid;


MultiValueSeq::MultiValueSeq(int numParts, int numFrames, double frameRate)
    : BaseSeqType("MultiValueSeq", numParts, numFrames, frameRate)
{

}


MultiValueSeq::MultiValueSeq(const MultiValueSeq& org)
    : BaseSeqType(org)
{

}


MultiValueSeq::~MultiValueSeq()
{

}


bool MultiValueSeq::loadPlainFormat(const std::string& filename)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        setIoErrorMessage(loader.errorMessage());
        return false;
    }

    setDimension(loader.numFrames(), loader.numParts());
    setFrameRate(1.0 / loader.timeStep());

    int i = 0;
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        copy((it->begin() + 1), it->end(), frame(i++).begin());
    }

    return true;
}


bool MultiValueSeq::saveAsPlainFormat(const std::string& filename)
{
    ofstream os(filename.c_str());
    os.setf(ios::fixed);

    if(!os){
        setIoErrorMessage(filename + " cannot be opened.");
        return false;
    }

    const int n = numFrames();
    const int m = numParts();
    const double r = frameRate();

    for(int i=0; i < n; ++i){
        os << (i / r);
        View v = frame(i);
        for(int j=0; j < m; ++j){
            os << " " << v[j];
        }
        os << "\n";
    }
    
    return true;
}


bool MultiValueSeq::write(YamlWriter& writer)
{
    bool result = false;
    
    writer.startMapping();

    if(BaseSeqType::write(writer)){

        writer.putKey("frames");
        writer.startSequence();
        const int n = numFrames();
        const int m = numParts();
        for(int i=0; i < n; ++i){
            writer.startFlowStyleSequence();
            View v = frame(i);
            for(int j=0; j < m; ++j){
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


bool MultiValueSeq::read(const YamlMapping& archive)
{
    if(BaseSeqType::read(archive) && (archive["type"].toString() == seqType())){
        const YamlSequence& values = *archive.findSequence("frames");
        if(values.isValid()){
            const int nParts = archive["numParts"].toInt();
            const int nFrames = values.size();
            setDimension(nFrames, nParts);
            for(int i=0; i < nFrames; ++i){
                const YamlSequence& frameNode = *values[i].toSequence();
                const int n = std::min(frameNode.size(), nParts);
                View v = frame(i);
                for(int j=0; j < n; ++j){
                    v[j] = frameNode[j].toDouble();
                }
            }
            return true;
        }
    }
    
    return false;
}
