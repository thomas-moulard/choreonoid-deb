/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MODEL_NODE_SET_H_INCLUDED
#define CNOID_BODY_MODEL_NODE_SET_H_INCLUDED

#include <Eigen/StdVector>
#include <cnoid/EigenTypes>
#include <cnoid/VrmlNodes>
#include <boost/shared_ptr.hpp>
#include <boost/signals.hpp>
#include "exportdecl.h"

namespace cnoid {

    class VrmlParser;

    class JointNodeSet;
    typedef boost::shared_ptr<JointNodeSet> JointNodeSetPtr;

    class JointNodeSet
    {
    public:
        typedef std::vector<Affine3, Eigen::aligned_allocator<Affine3> > Affine3Array;
        
        VrmlProtoInstancePtr jointNode;
        std::vector<JointNodeSetPtr> childJointNodeSets;
        Affine3Array transforms;
        std::vector<VrmlProtoInstancePtr> segmentNodes;
        std::vector<VrmlProtoInstancePtr> sensorNodes;
        std::vector<VrmlProtoInstancePtr> hwcNodes;
    };
    
    typedef std::vector<JointNodeSetPtr> JointNodeSetArray;

    class ModelNodeSetImpl;

    class CNOID_EXPORT ModelNodeSet
    {
      public:

        ModelNodeSet();
        virtual ~ModelNodeSet();

        bool loadModelFile(const std::string& filename);
		
        int numJointNodes();
        VrmlProtoInstancePtr humanoidNode();
        JointNodeSetPtr rootJointNodeSet();

        /**
           @if jp
           読み込み進行状況のメッセージを出力するためのシグナル.
           @note エラー発生時のメッセージはこのシグナルではなく例外によって処理される。
           @endif
        */
        boost::signal<void(const std::string& message)> sigMessage;

        class Exception {
        public:
            Exception(const std::string& description) : description(description) { }
            const char* what() const { return description.c_str(); }
        private:
            std::string description;
        };

      private:
        ModelNodeSetImpl* impl;
    };

    typedef boost::shared_ptr<ModelNodeSet> ModelNodeSetPtr;
};
    

#endif
