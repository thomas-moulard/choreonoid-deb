/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiAffine3Seq.h"
#include "PlainSeqFormatLoader.h"
#include "YamlNodes.h"
#include "YamlWriter.h"
#include "EigenUtil.h"
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

MultiAffine3Seq::MultiAffine3Seq(int numParts, int numFrames, double frameRate)
    : MultiAffine3Seq::BaseSeqType("MultiAffine3Seq", numParts, numFrames, frameRate)
{

}


MultiAffine3Seq::MultiAffine3Seq(const MultiAffine3Seq& org)
    : MultiAffine3Seq::BaseSeqType(org)
{

}


MultiAffine3Seq::~MultiAffine3Seq()
{

}


bool MultiAffine3Seq::loadPlainFormat(const std::string& filename)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        setIoErrorMessage(loader.errorMessage());
        return false;
    }

    if(loader.numParts() < 12){
        setIoErrorMessage(filename +
                          "does not have 12-columns "
                          "(3 for position vectors, 9 for attitde matrices)");
        return false;
    }
  
    setDimension(loader.numFrames(), 1);
    setTimeStep(loader.timeStep());

    int i = 0;
    View base = part(0);
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        vector<double>& data = *it;
        base[i].translation() << data[1], data[2], data[3];
        base[i].linear() <<
            data[ 4], data[ 5], data[ 6],
            data[ 7], data[ 8], data[ 9],
            data[10], data[11], data[12];
        ++i;
    }

    return true;
}


bool MultiAffine3Seq::saveTopPartAsPlainFormat(const std::string& filename)
{
    format f("%1$.4f");
    const int nFrames = numFrames();

    if(nFrames > 0 && numParts() > 0){

        ofstream os(filename.c_str());
        if(!os){
            setIoErrorMessage(filename + " cannot be opened.");
            return false;
        }

        const double r = frameRate();

        View base = part(0);
        for(int i=0; i < nFrames; ++i){
            os << (f % (i / r));
            const Affine3& T = base[i];
            for(int j=0; j < 3; ++j){
                os << " " << T.translation()[j];
            }
            for(int j=0; j < 3; ++j){
                for(int k=0; k < 3; ++k){
                    double m = T.linear()(j, k);
                    if(fabs(m) < 1.0e-14){
                        m = 0.0;
                    }
                    os << " " << m;
                }
            }
            os << " 0 0 0 0 0 0"; // velocity elements (dv, omega)
            os << "\n";
        }

        return true;
    }

    return false;
}


static inline void writeAffine3(YamlWriter& writer, const Affine3& value)
{
    writer.startFlowStyleSequence();

    writer.putScalar(value.translation().x());
    writer.putScalar(value.translation().y());
    writer.putScalar(value.translation().z());

    Vector3 rpy(rpyFromRot(value.linear()));
    writer.putScalar(rpy[0]);
    writer.putScalar(rpy[1]);
    writer.putScalar(rpy[2]);

    writer.endSequence();
}
    

bool MultiAffine3Seq::write(YamlWriter& writer)
{
    bool result = false;
    
    writer.startMapping();

    if(BaseSeqType::write(writer)){

        writer.putKeyValue("format", "XYZRPY");
    
        writer.putKey("frames");
        writer.startSequence();
        const int m = numParts();
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            View f = frame(i);
            writer.startFlowStyleSequence();
            for(int j=0; j < m; ++j){
                writeAffine3(writer, f[j]);
            }
            writer.endSequence();
        }
        writer.endSequence();

        result = true;
    }

    writer.endMapping();

    return result;
}


static void readAffine3(const YamlSequence& node, Affine3& out_value)
{
    if(node.size() == 6){

        Affine3::TranslationPart t = out_value.translation();
        t[0] = node[0].toDouble();
        t[1] = node[1].toDouble();
        t[2] = node[2].toDouble();

        const double r = node[3].toDouble();
        const double p = node[4].toDouble();
        const double y = node[5].toDouble();
        
        out_value.linear() = rotFromRpy(r, p, y);
    }
}


bool MultiAffine3Seq::read(const YamlMapping& archive)
{
    bool loaded = false;
    
    if(BaseSeqType::read(archive)){

        const string& type = archive["type"].toString();
        if((type == seqType() || type == "MultiSe3Seq") && (archive["format"].toString() == "XYZRPY")){

            const YamlSequence& values = *archive.findSequence("frames");

            if(values.isValid()){
                const int nParts = archive["numParts"].toInt();
                const int nFrames = values.size();
                setDimension(nFrames, nParts);
                
                for(int i=0; i < nFrames; ++i){
                    const YamlSequence& frameNode = *values[i].toSequence();
                    View f = frame(i);
                    const int n = std::min(frameNode.size(), nParts);
                    for(int j=0; j < n; ++j){
                        readAffine3(*frameNode[j].toSequence(), f[j]);
                    }
                }
                loaded = true;
            }
        }
    }

    return loaded;
}
