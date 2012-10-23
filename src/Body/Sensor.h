/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SENSOR_H_INCLUDED
#define CNOID_BODY_SENSOR_H_INCLUDED

#include <cnoid/EigenTypes>
#include <string>
#include <vector>
#include <iostream>
#include "exportdecl.h"

namespace cnoid {

    class Link;

    class CNOID_EXPORT Sensor
    {
      public:

        enum SensorType {
            COMMON = 0,
            FORCE,
            RATE_GYRO,
            ACCELERATION,
            PRESSURE,
            PHOTO_INTERRUPTER,
            VISION,
            TORQUE,
            RANGE,
            NUM_SENSOR_TYPES
        };

        static const int TYPE = COMMON;
		
        Sensor(); 
        virtual ~Sensor();

        static Sensor* create(int type);
        static void destroy(Sensor* sensor);

        virtual void operator=(const Sensor& org);

        virtual void clear();
		
        std::string name;
        int type;
        int id;
        Link* link;
        Matrix3 localR;
        Vector3 localPos;

        virtual void putInformation(std::ostream& os);

    };


    class CNOID_EXPORT ForceSensor : public Sensor
    {
      public:
        static const int TYPE = FORCE;
		
        ForceSensor();
        Vector3 f;
        Vector3 tau;

        virtual void clear();
        virtual void putInformation(std::ostream& os);
    };


    class CNOID_EXPORT RateGyroSensor : public Sensor
    {
      public:
        static const int TYPE = RATE_GYRO;

        RateGyroSensor();
        Vector3 w;

        virtual void clear();
        virtual void putInformation(std::ostream& os);
    };


    class CNOID_EXPORT AccelSensor : public Sensor
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        static const int TYPE = ACCELERATION;

        AccelSensor();

        Vector3 dv;

        virtual void clear();
        virtual void putInformation(std::ostream& os);

        // The following members are used in the ForwardDynamics class
        Vector2 x[3]; 
        bool isFirstUpdate;
    };

    class CNOID_EXPORT RangeSensor : public Sensor
    {
      public:
        static const int TYPE = RANGE;

        RangeSensor();

        double scanAngle, scanStep, scanRate, maxDistance;  
        std::vector<double> distances;
        double nextUpdateTime;
    };
};


#endif
