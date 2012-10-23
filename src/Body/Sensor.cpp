/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Sensor.h"
#include <cnoid/EigenUtil>

using namespace cnoid;

Sensor::Sensor()
{
    type = COMMON;
}


Sensor* Sensor::create(int type)
{
    Sensor* sensor;

    switch(type){
	
    case FORCE:
        sensor = new ForceSensor();
        break;
		
    case RATE_GYRO:
        sensor = new RateGyroSensor();
        break;
		
    case ACCELERATION:
        sensor = new AccelSensor();
        break;

    case RANGE:
        sensor = new RangeSensor();
        break;

    case PRESSURE:
    case PHOTO_INTERRUPTER:
    case VISION:
    case TORQUE:
        sensor = new Sensor();
        break;
    default:
        sensor = 0;
    }
	
    return sensor;
}


Sensor:: ~Sensor()
{
	
}


void Sensor::operator=(const Sensor& org)
{
    name     = org.name;
    type     = org.type;
    id       = org.id;
    localR   = org.localR;
    localPos = org.localPos;
}
	

void Sensor::destroy(Sensor* sensor)
{
    delete sensor;
}


void Sensor::clear()
{

}


void Sensor::putInformation(std::ostream &os)
{
    os << "name = " << name << ", id = " << id << "\n";
    os << "localAttitude = " << localR << ", localPos = " << localPos << std::endl;
}


ForceSensor::ForceSensor()
{
    type = FORCE;
}


void ForceSensor::clear()
{
    f.setZero();
    tau.setZero();
}


void ForceSensor::putInformation(std::ostream& os)
{
    os << "Force Sensor\n";
    Sensor::putInformation(os);
    os << "f = " << f << "tau = " << tau << std::endl;
}


RateGyroSensor::RateGyroSensor()
{
    type = RATE_GYRO;
}


void RateGyroSensor::clear()
{
    w.setZero();
}


void RateGyroSensor::putInformation(std::ostream& os)
{
    os << "Gyro\n";
    Sensor::putInformation(os);
    os << "omega = " << w << std::endl;
}


AccelSensor::AccelSensor()
{
    type = ACCELERATION;
    clear();
}


void AccelSensor::clear()
{
    dv << 0.0, 0.0, 9.8;
}


void AccelSensor::putInformation(std::ostream& os)
{
    os << "Acceleration Sensor\n";
    Sensor::putInformation(os);
    os << "dv = " << dv << std::endl;
}

RangeSensor::RangeSensor()
{
    type = RANGE;
    scanAngle = PI;
    scanStep = 0.1;
    scanRate = 10;
    maxDistance = 10;
    nextUpdateTime = 0;
}
