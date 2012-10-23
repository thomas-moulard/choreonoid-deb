/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "PronunSymbol.h"
#include <cnoid/YamlNodes>
#include <cnoid/YamlWriter>

using namespace std;
using namespace cnoid;


PronunSymbol::PronunSymbol()
{

}


PronunSymbol::PronunSymbol(const PronunSymbol& org)
    : PoseUnit(org),
      actualPoseUnit_(org.actualPoseUnit_)
{

}


PronunSymbol::~PronunSymbol()
{

}


PoseUnit* PronunSymbol::duplicate()
{
    return new PronunSymbol(*this);
}


bool PronunSymbol::restore(const YamlMapping& archive, const BodyPtr body)
{
    return true;
}


void PronunSymbol::store(YamlMapping& archive, const BodyPtr body) const
{
    archive.write("type", "PronunSymbol");
    archive.write("name", name(), YAML_DOUBLE_QUOTED);
}
