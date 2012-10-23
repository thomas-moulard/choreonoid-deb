/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_INTERPOLATOR_H_INCLUDED
#define CNOID_UTIL_INTERPOLATOR_H_INCLUDED

#include <vector>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT Interpolator
    {
      public:
        void clear();
        int appendSample(double x, double y);
        void setEndPoint(int sampleIndex, bool isNatural = false);
        int numSamples();
        bool update();
        void getDomain(double& out_lower, double& out_upper);
        double interpolate(double x);
        //bool interpolate(double x, double& out_y, double& out_d2, double& out_d3);
        //bool interpolate(double xBegin, double step, double* out_ybuf);
        //bool interpolate(double xBegin, double step, double* out_ybuf, double* out_d2buf, double* out_d3buf);
        
      private:

        int updateCubicSplineSegment(int begin);
        int updateSegmentBetweenTwoSamples(int begin);

        enum SegmentType { UNDETERMINED, CUBIC_SPLINE, POLYNOMINAL };

        struct Sample
        {
            double x;
            double y;
            double yp;
                
            // coefficients
            double a;
            double a_end;
            double b;
            double c;
                
            bool isEdge;
            bool isNaturalEdge;
            SegmentType segmentType;
        };

        std::vector<Sample> samples;
        int prevReferredSegments;
    };
}

#endif
