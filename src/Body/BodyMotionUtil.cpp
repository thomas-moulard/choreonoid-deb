/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionUtil.h"
#include "BodyMotion.h"
#include "Link.h"
#include "Sensor.h"
#include <cnoid/GaussianFilter>
#include <cnoid/RangeLimiter>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace boost;
using namespace cnoid;

static bool saveRootLinkAttAsRpyFormat(BodyMotion& motion, const std::string& filename, std::ostream& os)
{
    format f("%1$.4f %2$g %3$g %4$g\n");
    const MultiAffine3SeqPtr& linkPosSeq = motion.linkPosSeq();
    const int nFrames = linkPosSeq->numFrames();
        
    if(nFrames > 0 && linkPosSeq->numParts() > 0){
            
        ofstream ofs(filename.c_str());
        if(!ofs){
            os << filename + " cannot be opened.";
            return false;
        }
            
        const double r = linkPosSeq->frameRate();
        MultiAffine3Seq::View root = linkPosSeq->part(0);
        for(int i=0; i < nFrames; ++i){
            Vector3 rpy(rpyFromRot(root[i].linear()));
            for(int j=0; j < 3; ++j){
                if(fabs(rpy[j]) < 1.0e-14){
                    rpy[j] = 0.0;
                }
            }
            ofs << (f % (i / r) % rpy[0] % rpy[1] % rpy[2]);
        }
            
        return true;
    }
        
    return false;
}


static bool saveRootLinkAccAsGsensFile(BodyMotion& motion, BodyPtr body, const std::string& filename, std::ostream& os)
{
    if(!body){
        return false;
    }

    format f("%1$.4f %2$g %3$g %4$g\n");
    const MultiAffine3SeqPtr& linkPosSeq = motion.linkPosSeq();

    Sensor* gsens = 0;
    if(body->numSensors(Sensor::ACCELERATION) > 0){
        gsens = body->sensor(Sensor::ACCELERATION, 0);
    }
    if(!gsens || !gsens->link || gsens->link->index >= linkPosSeq->numParts()){
        return false;
    }

    const int nFrames = linkPosSeq->numFrames();
        
    if(nFrames > 0){
            
        ofstream ofs(filename.c_str());
        ofs.setf(ios::fixed);
        if(!ofs){
            os << filename + " cannot be opened.";
            return false;
        }

        Vector3Seq accSeq;
        calcLinkAccSeq(*linkPosSeq, gsens, 0, nFrames, accSeq);

        const double r = linkPosSeq->frameRate();
        for(int i=0; i < nFrames; ++i){
            Vector3 a = accSeq[i];
            for(int j=0; j < 3; ++j){
                if(fabs(a[j]) < 1.0e-14){
                    a[j] = 0.0;
                }
            }
            ofs << (f % (i / r) % a[0] % a[1] % a[2]);
        }
            
        return true;
    }
        
    return false;
}


void cnoid::calcLinkAccSeq
(MultiAffine3Seq& linkPosSeq, Sensor* gsens, int frameBegin, int numFrames, Vector3Seq& out_accSeq)
{
    const double r = linkPosSeq.frameRate();
    const double dt = 1.0 / r;
    vector<Vector3> vseq(numFrames);
    MultiAffine3Seq::View pseq = linkPosSeq.part(gsens->link->index);

    int last = numFrames - 1;
    for(int i=0; i < last; ++i){
        const Affine3& P0 = pseq[frameBegin + i];
        const Affine3& P1 = pseq[frameBegin + i + 1];
        Vector3 w = omegaFromRot(P0.linear().transpose() * P1.linear()) / dt;
        vseq[i].noalias() =
            (P1.translation() - P0.translation()) / dt + P0.linear() * w.cross(gsens->localPos);
    }
    vseq[last].setZero();
                
    out_accSeq.setNumFrames(numFrames);
    out_accSeq[0].noalias() = (pseq[frameBegin].linear().transpose() * vseq[0]) / dt;
    for(int i=1; i < numFrames; ++i){
        out_accSeq[i].noalias() =
            pseq[frameBegin + i].linear().transpose() * (vseq[i] - vseq[i-1]) / dt;
    }
}

    
bool cnoid::loadHrpsysSeqFileSet(BodyMotion& motion, const std::string& filename, std::ostream& os)
{
    motion.setNumFrames(0);

    filesystem::path orgpath(filename);
    
    bool loaded = false;
    
    filesystem::path posFile = filesystem::change_extension(orgpath, ".pos");
    if(filesystem::exists(posFile) && !filesystem::is_directory(posFile)){
        string posFileString(posFile.file_string());
        if(motion.jointPosSeq()->loadPlainFormat(posFileString)){
            if(posFileString == filename){
                loaded = true;
            }
        }
    }
    filesystem::path waistFile = filesystem::change_extension(orgpath, ".waist");
    if(filesystem::exists(waistFile) && !filesystem::is_directory(waistFile)){
        string waistFileString(waistFile.file_string());
        if(motion.linkPosSeq()->loadPlainFormat(waistFileString)){
            if(waistFileString == filename){
                loaded = true;
            }
        }
    }
    filesystem::path zmpFile = filesystem::change_extension(orgpath, ".zmp");
    if(filesystem::exists(zmpFile) && !filesystem::is_directory(zmpFile)){
        string zmpFileString(zmpFile.file_string());
        if(motion.relativeZmpSeq()->loadPlainFormat(zmpFileString)){
            if(zmpFileString == filename){
                loaded = true;
            }
            
        }
    }

    if(!loaded){
        motion.setNumFrames(0);
    }
    
    return loaded;
}


bool cnoid::saveHrpsysSeqFileSet(BodyMotion& motion, BodyPtr body, const std::string& filename, std::ostream& os)
{
    filesystem::path orgpath(filename);
    filesystem::path bpath(orgpath.branch_path() / filesystem::path(basename(orgpath)));

    if(motion.jointPosSeq()->saveAsPlainFormat(filesystem::change_extension(orgpath, ".pos").file_string()) &&
       motion.linkPosSeq()->saveTopPartAsPlainFormat(filesystem::change_extension(orgpath, ".waist").file_string()) &&
       saveRootLinkAttAsRpyFormat(motion, filesystem::change_extension(orgpath, ".hip").file_string(), os)){

        saveRootLinkAccAsGsensFile(motion, body, filesystem::change_extension(orgpath, ".gsens").file_string(), os);
        
        if(motion.hasRelativeZmpSeq()){
            return motion.relativeZmpSeq()->saveAsPlainFormat(filesystem::change_extension(orgpath, ".zmp").file_string());
        }
        return true;
    }
    return false;
}


static void applyPollardVelocityLimitFilterSub
(MultiValueSeq::View seq, double deltaUVLimit, double deltaLVLimit, double deltaKs,
 vector<double>& forward, vector<double>& backward)
{
    const double sqrtDeltaKs = sqrt(deltaKs);
    const int n = seq.size();
    const int last = n - 1;
    
    double v0, vv0, vv1; 
    double aa1;

    // forward pass
    forward[0] = seq[0];
    v0 = 0.0;
    vv0 = 0.0;
    for(int i = 0; i < last; ++i){
        aa1 = 2.0 * sqrtDeltaKs * (v0 - vv0) + deltaKs * (seq[i] - forward[i]);
        vv1 = vv0 + aa1;
        if(vv1 > deltaUVLimit){
            vv1 = deltaUVLimit;
        } else if(vv1 < deltaLVLimit) {
            vv1 = deltaLVLimit;
        }
        forward[i+1] = forward[i] + vv1;
        v0 = seq[i+1] - seq[i];
        vv0 = vv1;
    }
    
    // backward pass
    v0 = 0.0;
    vv0 = 0.0;
    backward[last] = seq[last];
    for(int i = last; i > 0; --i) {
        aa1 = 2.0 * sqrtDeltaKs * (v0 - vv0) + deltaKs * (seq[i] - backward[i]);
        vv1 = vv0 + aa1;
        if(vv1 > -deltaLVLimit){
            vv1 = -deltaLVLimit;
        } else if(vv1 < -deltaUVLimit){
            vv1 = -deltaUVLimit;
        }
        backward[i-1] = backward[i] + vv1;
        v0 = seq[i-1] - seq[i];
        vv0 = vv1;
    }
    
    // average the forward and backward passes 
    for(int i=0; i < n; i++){
        seq[i] = 0.5 * (forward[i] + backward[i]);
    }
}


static void applyVelocityLimitFilterSub
(MultiValueSeq::View seq, double deltaUVLimit, double deltaLVLimit,
 vector<double>& forward, vector<double>& backward)
{
    const int n = seq.size();
    const int last = n - 1;

    // forward pass
    forward[0] = seq[0];
    for(int i = 0; i < last; ++i){
        double v = (seq[i+1] - forward[i]);
        if(v > deltaUVLimit){
            v = deltaUVLimit;
        } else if(v < deltaLVLimit){
            v = deltaLVLimit;
        }
        forward[i+1] = forward[i] + v;
    }
    
    // backward pass
    backward[last] = seq[last];
    for(int i = last; i > 0; --i) {
        double v = (seq[i-1] - backward[i]);
        if(v > -deltaLVLimit){
            v = -deltaLVLimit;
        } else if(v < -deltaUVLimit){
            v = -deltaUVLimit;
        }
        backward[i-1] = backward[i] + v;
    }
    
    // average the forward and backward passes 
    for(int i=0; i < n; i++){
        seq[i] = 0.5 * (forward[i] + backward[i]);
    }
}


bool cnoid::applyVelocityLimitFilter2(MultiValueSeq& seq, int part, double absLimit)
{
    cout << "applyVelocityLimitFilter(seq," << part << ", " << absLimit << ")" << endl;
    const int numFrames = seq.numFrames();
    const double frameRate = seq.frameRate();
    vector<double> forward(numFrames);
    vector<double> backward(numFrames);
    const double deltaUVLimit = absLimit / frameRate;
    const double deltaLVLimit = -absLimit / frameRate;
    applyVelocityLimitFilterSub(seq.part(part), deltaUVLimit, deltaLVLimit, forward, backward);

    return true;
}


bool cnoid::applyVelocityLimitFilterDummy()
{
    cout << "applyVelocityLimitFilterDummy()" << endl;
	return false;
}


static bool applyVelocityLimitFilterMain
(MultiValueSeq& seq, BodyPtr body, double ks, bool usePollardMethod, std::ostream& os)
{
    bool applied = false;
    
    os << "applying the velocity limit filter ..." << endl;
    
    const int numParts = seq.numParts();
    const int numFrames = seq.numFrames();
    const double frameRate = seq.frameRate();
    const double deltaKs = ks / frameRate;
    
    vector<double> forward(numFrames);
    vector<double> backward(numFrames);
    
    int n = std::min(numParts, body->numJoints());
    for(int i=0; i < n; ++i){
        Link* joint = body->joint(i);
        if(joint->uvlimit != std::numeric_limits<double>::max() ||
           joint->lvlimit != -std::numeric_limits<double>::max()){
            const double deltaUVLimit = joint->uvlimit / frameRate;
            const double deltaLVLimit = joint->lvlimit / frameRate;
            
            os << str(format(" seq %1%: lower limit = %2%, upper limit = %3%")
                      % i % joint->lvlimit % joint->uvlimit) << endl;

            if(usePollardMethod){
                applyPollardVelocityLimitFilterSub(
                    seq.part(i), deltaUVLimit, deltaLVLimit, deltaKs, forward, backward);
            } else {
                applyVelocityLimitFilterSub(
                    seq.part(i), deltaUVLimit, deltaLVLimit, forward, backward);
            }
                    
            applied = true;
        }
    }
    return applied;
}


bool cnoid::applyPollardVelocityLimitFilter
(MultiValueSeq& seq, BodyPtr body, double ks, std::ostream& os)
{
    return applyVelocityLimitFilterMain(seq, body, ks, true, os);
}


bool cnoid::applyVelocityLimitFilter
(MultiValueSeq& seq, BodyPtr body, std::ostream& os)
{
    return applyVelocityLimitFilterMain(seq, body, 1.0, false, os);
}


void cnoid::applyGaussianFilter
(MultiValueSeq& seq, double sigma, int range, std::ostream& os)
{
    vector<double> gwin;
    setGaussWindow(sigma, range, gwin);
    
    vector<double> orgseq(seq.numFrames());
    
    for(int i=0; i < seq.numParts(); ++i){
        if(i==0){
            os << str(format("applying the gaussian filter (sigma = %1%, range = %2%) to seq") %
                      sigma % range) << endl;
        }
        os << " " << i;

        MultiValueSeq::View part = seq.part(i);
        std::copy(part.begin(), part.end(), orgseq.begin());
        applyGaussianFilter(part, orgseq, gwin, 0.0);
    }
}


void cnoid::applyRangeLimitFilter
(MultiValueSeq& seq, BodyPtr body, double limitGrad, double edgeGradRatio, double margin, std::ostream& os)
{
    RangeLimiter limiter;

    os << "applying the joint position range limit filter ..." << endl;
    
    const int numParts = seq.numParts();
    
    int n = std::min(numParts, body->numJoints());
    for(int i=0; i < n; ++i){
        Link* joint = body->joint(i);
        if(joint->ulimit != std::numeric_limits<double>::max() ||
           joint->llimit != -std::numeric_limits<double>::max()){
            const double ulimit = joint->ulimit - margin;
            const double llimit = joint->llimit + margin;
            if(ulimit > llimit){
                os << str(format(" seq %1%: lower limit = %2%, upper limit = %3%")
                          % i % joint->llimit % joint->ulimit) << endl;

                MultiValueSeq::View part = seq.part(i);
                limiter.apply(part, ulimit, llimit, limitGrad, edgeGradRatio);
            }
        }
    }
}
