/*
  @author Shin'ichiro Nakaoka
*/

#include "LinkGroup.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/YamlNodes>

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace cnoid;


LinkGroup::LinkGroup()
{

}


LinkGroup::~LinkGroup()
{

}


/**
   @param linkGroupSeq YAML node defining a ling group set.
   If linkGroupSeq.isValid() is false, a whole body group that contains all the links is created.
*/
LinkGroupPtr LinkGroup::create(BodyPtr body, const YamlSequence& linkGroupSeq)
{
    LinkGroupPtr group = new LinkGroup();
    group->setName("Whole Body");
    if(!linkGroupSeq.isValid() || !group->load(body, linkGroupSeq)){
        group->setFlatLinkList(body);
    }
    return group;
}


bool LinkGroup::load(BodyPtr& body, const YamlSequence& linkGroupSeq)
{
    for(int i=0; i < linkGroupSeq.size(); ++i){

        const YamlNode& node = linkGroupSeq[i];
        YamlNodeType type = node.type();

        if(type == YAML_SCALAR){
            Link* link = body->link(node.toString());
            if(!link){
                return false;
            }
            elements.push_back(link->index);

        } else if(type == YAML_MAPPING){
            const YamlMapping& group = *node.toMapping();
            LinkGroupPtr linkGroup = new LinkGroup();
            linkGroup->setName(group["name"]);
            if(linkGroup->load(body, *group["links"].toSequence())){
                elements.push_back(linkGroup);
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    return true;
}


void LinkGroup::setFlatLinkList(BodyPtr& body)
{
    for(int i=0; i < body->numLinks(); ++i){
        elements.push_back(i);
    }
}

