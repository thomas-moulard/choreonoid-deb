/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_LOADER_H_INCLUDED
#define CNOID_BODY_BODY_LOADER_H_INCLUDED

#include "Body.h"
#include <cnoid/SignalProxy>
#include "exportdecl.h"

namespace cnoid {

    class ModelNodeSet;
    typedef boost::shared_ptr<ModelNodeSet> ModelNodeSetPtr;
    
    class BodyLoaderImpl;
  
    class CNOID_EXPORT BodyLoader
    {
      public:
        BodyLoader();
        ~BodyLoader();

        void setDivisionNumber(int n);
    
        BodyPtr loadModelFile(
            const std::string& filename,
            bool doTriangulation = true, bool doNormalGeneration = true, bool createColdetModel = true);

        const std::string& errorMessage();
    
        ModelNodeSetPtr modelNodeSet();

        SignalProxy< boost::signal<void(const std::string& message)> > sigMessage();
    
      private:
        BodyLoaderImpl* impl;
    };
}

#endif
