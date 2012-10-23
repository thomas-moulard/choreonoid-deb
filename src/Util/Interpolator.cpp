/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Interpolator.h"
#include <cmath>
#include <limits>

using namespace std;
using namespace cnoid;

void Interpolator::clear()
{
    samples.clear();
    prevReferredSegments = -1;
}


int Interpolator::numSamples()
{
    return samples.size();
}


int Interpolator::appendSample(double x, double y)
{
    if(!samples.empty()){
        Sample& prev = samples.back();
        if(fabs(prev.x - x) < std::numeric_limits<double>::epsilon()){
            samples.pop_back();
        }
    }
    
    Sample s;
    s.x = x;
    s.y = y;
    s.yp = 0.0;
    s.isEdge = false;
    s.isNaturalEdge = false;
    s.segmentType = UNDETERMINED;
    
    int index = samples.size();
    samples.push_back(s);

    return index;
}


void Interpolator::setEndPoint(int sampleIndex, bool isNatural)
{
    Sample& sample = samples[sampleIndex];
    sample.isEdge = true;
    sample.isNaturalEdge = isNatural;
    sample.yp = 0.0; // first-order derivative (velocity)
}


bool Interpolator::update()
{
    int n = samples.size();

    int s = 0;
    while(true){
        const int m = n - s;
        if(m < 2){
            break;
        }
        if(m >= 3 && !samples[s + 1].isEdge){
            s = updateCubicSplineSegment(s);
        } else {
            s = updateSegmentBetweenTwoSamples(s);
        }
    }

    return (n >= 2);
}


int Interpolator::updateCubicSplineSegment(int begin)
{
    Sample& s0 = samples[begin];
    s0.segmentType = CUBIC_SPLINE;
    s0.isEdge = true;

    if(s0.isNaturalEdge){
        s0.a = 0.0;
        s0.b = 0.0;
    } else {
        Sample& s1 = samples[begin + 1];
        s0.a = -0.5;
        s0.b = (3.0 / (s1.x - s0.x)) * ((s1.y - s0.y) / (s1.x - s0.x) - s0.yp);
    }

    const int n = samples.size();
    int i = (begin + 1);
    while(true) {
        Sample& s0 = samples[i-1];
        Sample& s1 = samples[i];
        Sample& s2 = samples[i+1];

        s1.segmentType = CUBIC_SPLINE;
        double sig = (s1.x - s0.x) / (s2.x - s0.x);
        double p = sig * s0.a + 2.0;
        s1.a = (sig - 1.0) / p;
        double b = (s2.y - s1.y) / (s2.x - s1.x) - (s1.y - s0.y) / (s1.x - s0.x);
        s1.b = (6.0 * b / (s2.x - s0.x) - sig * s0.b) / p;

        if(s2.isEdge || i == (n - 2)){
            break;
        }

        ++i;
    }

    double qf;
    double bf;
    Sample& sf0 = samples[i];
    Sample& sf = samples[i+1];
    const int next = i + 1;

    sf.isEdge = true;

    if(sf.isNaturalEdge){
        qf = 0.0;
        bf = 0.0;
    } else {
        qf = 0.5;
        bf = (3.0 / (sf.x - sf0.x)) * (sf.yp - (sf.y - sf0.y) / (sf.x - sf0.x));
    }

    const double a_save = sf.a;
    sf.a = (bf - qf * sf0.b) / (qf * sf0.a + 1.0);

    while(i >= begin){
        Sample& s0 = samples[i];
        Sample& s1 = samples[i+1];
        s0.a = s0.a * s1.a + s0.b;
        --i;
    }

    sf.a_end = sf.a;
    sf.a = a_save;

    return next;
}


int Interpolator::updateSegmentBetweenTwoSamples(int begin)
{
    Sample& s0 = samples[begin];
    s0.segmentType = POLYNOMINAL;
    s0.isEdge = true;
    Sample& s1 = samples[begin+1];
    s1.isEdge = true;

    const double h = (s1.x - s0.x);
    const double h2 = h * h;
    const double h3 = h2 * h;

    const double d0 = s0.isEdge ? s0.yp : 0.0;
    const double d1 = s1.isEdge ? s1.yp : 0.0;

    s0.a = d0;
    s0.b = 3.0 * (s1.y - s0.y) / h2 - (2.0 * d0 - d1) / h;
    s0.c = (d0 + d1) / h2 + 2.0 * (s0.y - s1.y) / h3;
    
    return begin + 1;
}


void Interpolator::getDomain(double& out_lower, double& out_upper)
{
    if(samples.empty()){
        out_lower = 0.0;
        out_upper = 0.0;
    } else {
        out_lower = samples.front().x;
        out_upper = samples.back().x;
    }
}


double Interpolator::interpolate(double x)
{
    int lower;
    int upper;
    
    const int n = samples.size();
    int k;
    if(prevReferredSegments >= 0 && prevReferredSegments < (n - 1)){
        k = prevReferredSegments;
        if(x >= samples[k].x && x < samples[k+1].x){
            goto calc;
        }
    }

    lower = 0;
    upper = n - 1;

    if(x < samples[0].x){
        return samples[0].y;
    } else if(x >= samples[upper].x){
        return samples[upper].y;
    }

    while(upper - lower > 1){
        k = (upper + lower) / 2;
        if(samples[k].x > x){
            upper = k;
        } else {
            lower = k;
        }
    }

    k = lower;

  calc:

    prevReferredSegments = k;
    const Sample& s0 = samples[k];
    const Sample& s1 = samples[k+1];

    if(s0.segmentType == CUBIC_SPLINE){
        const double a_end = (s1.isEdge ? s1.a_end : s1.a);
        const double h = s1.x - s0.x;
        const double A = (s1.x - x) / h;
        const double B = (x - s0.x) / h;
        return A*s0.y + B*s1.y + ((A*A*A - A) * s0.a + (B*B*B - B) * a_end) * (h*h) / 6.0;

    } else if(s0.segmentType == POLYNOMINAL){
        const double& a0 = s0.y;
        const double& a1 = s0.a;
        const double& a2 = s0.b;
        const double& a3 = s0.c;
        const double h = x - s0.x;
        const double h2 = h * h;
        const double h3 = h2 * h;
        return (a0 + a1 * h + a2 * h2 + a3 * h3);
    }

    return 0.0;
}


/*
bool interpolate(double x, double& out_y, double& out_d2, double& out_d3);
bool interpolate(double xBegin, double step, double* out_ybuf);
bool interpolate(double xBegin, double step, double* out_ybuf, double* out_d2buf, double* out_d3buf);
*/
