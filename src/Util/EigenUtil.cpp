
#include "EigenUtil.h"

namespace cnoid {

    Matrix3 rotFromRpy(double r, double p, double y)
    {
        const double cr = cos(r);
        const double sr = sin(r);
        const double cp = cos(p);
        const double sp = sin(p);
        const double cy = cos(y);
        const double sy = sin(y);

        Matrix3 R;
        R << cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy,
             cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy,
             -sp  , sr*cp           , cr*cp;

        return R;
    }


    Vector3 omegaFromRot(const Matrix3& R)
    {
        double alpha = (R(0,0) + R(1,1) + R(2,2) - 1.0) / 2.0;

        if(fabs(alpha - 1.0) < 1.0e-6) {   //th=0,2PI;
            return Vector3::Zero();

        } else {
            double th = acos(alpha);
            double s = sin(th);

            if (s < std::numeric_limits<double>::epsilon()) {   //th=PI
                return Vector3( sqrt((R(0,0)+1)*0.5)*th, sqrt((R(1,1)+1)*0.5)*th, sqrt((R(2,2)+1)*0.5)*th );
            }

            double k = -0.5 * th / s;

            return Vector3((R(1,2) - R(2,1)) * k,
                           (R(2,0) - R(0,2)) * k,
                           (R(0,1) - R(1,0)) * k);
        }
    }
}
