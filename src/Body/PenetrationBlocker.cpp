/**
   @author Shin'ichiro Nakaoka
*/

#include "PenetrationBlocker.h"
#include <cnoid/ColdetModelPair>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

    class PenetrationBlockerImpl
    {
    public:
        PenetrationBlockerImpl(Link* targetLink);

        Link* targetLink;
        ColdetModelPtr targetModel;
        std::vector<ColdetModelPairPtr> coldetModelPairs;
        double targetDepth;
        Vector3 pPrevGiven;
        bool isPrevBlocked;
        Vector3 prevBlockedPosition;
        Vector3 prevBlockedNormal;

        struct LinkInfo {
            Link* link;
            // A coldet model is copied from the original one
            ColdetModelPtr coldetModel;
        };
        std::vector<LinkInfo> opponentLinkInfos;

        void addOpponentLink(Link* link);

        void start();
        bool adjust(Vector3& io_p, const Matrix3& R, const Vector3& pushDirection);
        bool adjustSub(Vector3& io_p, const Matrix3& R, const Vector3& pushDirection);
    };
}


PenetrationBlocker::PenetrationBlocker(Link* targetLink)
{
    impl = new PenetrationBlockerImpl(targetLink);
}


PenetrationBlockerImpl::PenetrationBlockerImpl(Link* targetLink)
    : targetLink(targetLink)
{
    if(targetLink->coldetModel){
        targetModel = new ColdetModel(*targetLink->coldetModel);
    }
    pPrevGiven = targetLink->p;

    targetDepth = 0.001;
    isPrevBlocked = false;
}


void PenetrationBlocker::addOpponentLink(Link* link)
{
    impl->addOpponentLink(link);
}


void PenetrationBlockerImpl::addOpponentLink(Link* link)
{
    if(link->coldetModel){
        ColdetModelPtr opponendModel = new ColdetModel(*link->coldetModel);
        coldetModelPairs.push_back(new ColdetModelPair(opponendModel, targetModel));
        LinkInfo info;
        info.link = link;
        info.coldetModel = opponendModel;
        opponentLinkInfos.push_back(info);
    }
}


void PenetrationBlocker::setDepth(double depth)
{
    impl->targetDepth = depth;
}


void PenetrationBlocker::start()
{
    impl->isPrevBlocked = false;
}


bool PenetrationBlocker::adjust(Vector3& io_p, const Matrix3& R, const Vector3& pushDirection)
{
    return impl->adjust(io_p, R, pushDirection);
}


bool PenetrationBlockerImpl::adjust(Vector3& io_p, const Matrix3& R, const Vector3& pushDirection)
{
    if(isPrevBlocked){
        if((io_p - prevBlockedPosition).dot(prevBlockedNormal) < 0.0){
            io_p = prevBlockedPosition;
            return true;
        }
    }
    return adjustSub(io_p, R, pushDirection);
}


bool PenetrationBlockerImpl::adjustSub(Vector3& io_p, const Matrix3& R, const Vector3& pushDirection)
{
    bool blocked = false;
    const Vector3 s = pushDirection.normalized();
    
    for(size_t i=0; i < opponentLinkInfos.size(); ++i){
        LinkInfo& info = opponentLinkInfos[i];
        info.coldetModel->setPosition(info.link->R, info.link->p);
    }

    int loop;
    Vector3 maxnormal(Vector3::Zero());
    
    for(loop = 0; loop < 100; ++loop){

        targetModel->setPosition(R, io_p);

        double maxsdepth = 0.0;
        double maxdepth = 0.0;
        
        for(size_t i=0; i < coldetModelPairs.size(); ++i){
            ColdetModelPair& modelPair = *coldetModelPairs[i];
            const std::vector<collision_data>& cols = modelPair.detectCollisions();
            for(size_t j=0; j < cols.size(); ++j){
                const collision_data& col = cols[j];
                if(col.depth > targetDepth){
                    double d = -col.n_vector.dot(s);
                    if(d > 0.0){
                        double sdepth = col.depth * d;
                        if(sdepth > maxsdepth){
                            maxsdepth = sdepth;
                            maxdepth = col.depth;
                            maxnormal = col.n_vector;
                        }
                    }
                }
            }
        }
        
        if(maxsdepth > 0.0){
            io_p += (maxdepth - targetDepth) * maxnormal;
            blocked = true;
        } else {
            break;
        }
    }

    isPrevBlocked = blocked;
    prevBlockedPosition = io_p;
    prevBlockedNormal = maxnormal;

    return blocked;
}
