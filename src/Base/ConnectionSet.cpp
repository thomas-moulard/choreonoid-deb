/**
   @author Shin'ichiro Nakaoka
*/

#include "ConnectionSet.h"

using namespace cnoid;


ConnectionSet::ConnectionSet()
{

}


ConnectionSet::ConnectionSet(const ConnectionSet& org)
{
    add(org);
}


/**
   This operator disconnects existing connections.
*/
ConnectionSet& ConnectionSet::operator=(const ConnectionSet& org)
{
    disconnect();
    add(org);
    return *this;
}
        

/**
   Destructor.
   Note that the connections are *not* disconnected by the destructor.
   This design is employed to  allow a use of the copy constructor and copy operator.
*/
ConnectionSet::~ConnectionSet()
{

}


void ConnectionSet::disconnect()
{
    for(size_t i=0; i < boostConnections.size(); ++i){
        boostConnections[i].disconnect();
    }
    boostConnections.clear();
}
        

void ConnectionSet::add(const boost::signals::connection& connection)
{
    boostConnections.push_back(connection);
}
        

void ConnectionSet::add(const ConnectionSet& connections)
{
    for(size_t i=0; i < connections.boostConnections.size(); ++i){
        boostConnections.push_back(connections.boostConnections[i]);
    }
}
        

void ConnectionSet::block()
{
    for(size_t i=0; i < boostConnections.size(); ++i){
        boostConnections[i].block();
    }
}
        

void ConnectionSet::unblock()
{
    for(size_t i=0; i < boostConnections.size(); ++i){
        boostConnections[i].unblock();
    }
}
