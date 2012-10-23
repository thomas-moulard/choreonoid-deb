/*
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_GROUP_H_INCLUDED
#define CNOID_BODY_LINK_GROUP_H_INCLUDED

#include <boost/variant.hpp>
#include <cnoid/Referenced>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

    class YamlSequence;
    
    class Body;
    typedef boost::intrusive_ptr<Body> BodyPtr;

    class LinkGroup;
    typedef boost::intrusive_ptr<LinkGroup> LinkGroupPtr;

    class CNOID_EXPORT LinkGroup : public Referenced
    {
        typedef boost::variant<LinkGroupPtr, int> Element;
            
      public:

        static LinkGroupPtr create(BodyPtr body, const YamlSequence& linkGroupSeq);

        LinkGroup();
        virtual ~LinkGroup();

        inline void setName(const std::string& name) { name_ = name; }
        inline const std::string& name() { return name_; }

        inline int numElements() { return elements.size(); }
        inline bool isSubGroup(int index) { return elements[index].which() == 0; }
        inline bool isLinkIndex(int index) { return elements[index].which() == 1; }
        inline LinkGroupPtr subGroup(int index) { return boost::get<LinkGroupPtr>(elements[index]); }
        inline int linkIndex(int index) { return boost::get<int>(elements[index]); }

        std::vector<int> collectLinkIndices() const;
        std::vector<LinkGroupPtr> collectGroups() const;

      private:

        std::string name_;
        std::vector<Element> elements;

        bool load(BodyPtr& body, const YamlSequence& linkGroupseq);
        void setFlatLinkList(BodyPtr& body);
    };
}


#endif
