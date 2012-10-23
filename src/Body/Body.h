/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_H_INCLUDED
#define CNOID_BODY_BODY_H_INCLUDED

#include "LinkTraverse.h"
#include "LinkGroup.h"
#include <cnoid/YamlNodes>
#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <boost/shared_ptr.hpp>
#include <map>
#include "exportdecl.h"

namespace cnoid {

    class Sensor;

    class Body;
    typedef boost::intrusive_ptr<Body> BodyPtr;

    class JointPath;
    typedef boost::shared_ptr<JointPath> JointPathPtr;

    class InverseKinematics;
    typedef boost::shared_ptr<InverseKinematics> InverseKinematicsPtr;

    class LinkGroup;

    class YamlMapping;
    typedef boost::intrusive_ptr<YamlMapping> YamlMappingPtr;

    struct BodyHandleEntity {
        Body* body;
    };

    struct BodyInterface;
    struct BodyCustomizerInterface;
    typedef void* BodyHandle;
    typedef void* BodyCustomizerHandle;

    class CNOID_EXPORT Body : public Referenced
    {
      public:

        static void addCustomizerDirectory(const std::string& path);

        static BodyInterface* bodyInterface();

        Body();
        virtual ~Body();

        virtual BodyPtr duplicate() const;

        inline const std::string& name() {
            return name_;
        }
        inline void setName(const std::string& name) {
            name_ = name;
        }
        inline const std::string& modelName() {
            return modelName_;
        }
        inline void setModelName(const std::string& name) {
            modelName_ = name;
        }
        
        void setRootLink(Link* link);

        /**
           This function must be called when the structure of the link tree is changed.
        */
        void updateLinkTree();

        /**
           The number of the links that work as a joint.
           Note that the acutal value is the maximum joint ID plus one.
           Thus there may be a case where the value does not correspond
           to the actual number of the joint-links.
           In other words, the value represents the size of the link sequence
           obtained by joint() function.
        */
        inline int numJoints() const {
            return jointIdToLinkArray.size();
        }

        /**
           This function returns a link that has a given joint ID.
           If there is no link that has a given joint ID,
           the function returns a dummy link object whose ID is minus one.
           The maximum id can be obtained by numJoints().
        */
        inline Link* joint(int id) const {
            return jointIdToLinkArray[id];
        }

        /**
           The vector<Link*> corresponding to the sequence of joint().
        */
        inline const std::vector<Link*>& joints() const {
            return jointIdToLinkArray;
        }

        /**
           The number of all the links the body has.
           The value corresponds to the size of the sequence obtained by link() function.
        */
        inline int numLinks() const {
            return linkTraverse_.numLinks();
        }

        /**
           This function returns the link of a given index in the whole link sequence.
           The order of the sequence corresponds to a link-tree traverse from the root link.
           The size of the sequence can be obtained by numLinks().
        */
        inline Link* link(int index) const {
            return linkTraverse_.link(index);
        }

        inline const LinkTraverse& links() const {
            return linkTraverse_;
        }

        /**
           LinkTraverse object that traverses all the links from the root link
        */
        inline const LinkTraverse& linkTraverse() const {
            return linkTraverse_;
        }

        /**
           This function returns a link that has a given name.
        */
        Link* link(const std::string& name) const;

        /**
           The root link of the body
        */
        inline Link* rootLink() const {
            return rootLink_;
        }

        // sensor access methods
        Sensor* createSensor(Link* link, int sensorType, int id, const std::string& name);

        void addSensor(Sensor* sensor, int sensorType, int id );

        inline Sensor* sensor(int sensorType, int sensorId) const {
            return allSensors[sensorType][sensorId];
        }

        inline int numSensors(int sensorType) const {
            return allSensors[sensorType].size();
        }

        inline int numSensorTypes() const {
            return allSensors.size();
        }

        void clearSensorValues();

        template <class TSensor> inline TSensor* sensor(int id) const {
            return static_cast<TSensor*>(allSensors[TSensor::TYPE][id]);
        }

        template <class TSensor> inline TSensor* sensor(const std::string& name) const {
            TSensor* sensor = 0;
            NameToSensorMap::const_iterator p = nameToSensorMap.find(name);
            if(p != nameToSensorMap.end()){
                sensor = dynamic_cast<TSensor*>(p->second);
            }
            return sensor;
        }

        /**
           This function returns true when the whole body is a static, fixed object like a floor.
        */
        inline bool isStaticModel() {
            return isStaticModel_;
        }

        double calcTotalMass();

        inline double totalMass() const {
            return totalMass_;
        }

        Vector3 calcCM();
        const Vector3& lastCM() { return lastCM_; }

        void calcTotalMomentum(Vector3& out_P, Vector3& out_L);
        void setDefaultRootPosition(const Vector3& p, const Matrix3& R);
        void getDefaultRootPosition(Vector3& out_p, Matrix3& out_R);
        void initializeConfiguration();
        void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false);
        void clearExternalForces();
        JointPathPtr getJointPath(Link* baseLink, Link* targetLink);
        void setVirtualJointForces();

        virtual InverseKinematicsPtr getDefaultIK(Link* targetLink);
        
        /**
           This function must be called before the collision detection.
           It updates the positions and orientations of the models
           for detecting collisions between links.
        */
        void updateLinkColdetModelPositions();

        void putInformation(std::ostream &out);

        bool installCustomizer();
        bool installCustomizer(BodyCustomizerInterface* customizerInterface);

        struct LinkConnection {
            Link* link[2];
            Vector3 point[2];
            int numConstraintAxes;
            Vector3 constraintAxes[3];
        };
        typedef std::vector<LinkConnection> LinkConnectionArray;

        LinkConnectionArray linkConnections;

        YamlMapping* info() { return info_.get(); }
        void resetInfo(YamlMappingPtr info);
        
        LinkGroup* linkGroup() { return linkGroup_.get(); }

      protected:

        Body(const Body& org);

        virtual void doResetInfo(const YamlMapping& info);

      private:

        bool isStaticModel_;
        Link* rootLink_;

        std::string name_;
        std::string modelName_;

        typedef std::vector<Link*> LinkArray;
        LinkArray jointIdToLinkArray;

        LinkTraverse linkTraverse_;

        typedef std::map<std::string, Link*> NameToLinkMap;
        NameToLinkMap nameToLinkMap;

        // sensor = sensors[type][sensorId]
        typedef std::vector<Sensor*> SensorArray;
        std::vector<SensorArray> allSensors;

        typedef std::map<std::string, Sensor*> NameToSensorMap;
        NameToSensorMap nameToSensorMap;

        double totalMass_;
        Vector3 lastCM_;

        Vector3 defaultRootPosition;
        Matrix3 defaultRootAttitude;

        LinkGroupPtr linkGroup_;
        YamlMappingPtr info_;

        // Members for customizer
        BodyCustomizerHandle customizerHandle;
        BodyCustomizerInterface* customizerInterface;
        BodyHandleEntity bodyHandleEntity;
        BodyHandle bodyHandle;

        void initialize();
        Link* createEmptyJoint(int jointId);
        void setVirtualJointForcesSub();

        friend class CustomizedJointPath;
    };

};


CNOID_EXPORT std::ostream &operator<< (std::ostream& out, cnoid::Body& body);

#endif
