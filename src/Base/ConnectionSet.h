/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_CONNECTION_SET_H_INCLUDED
#define CNOID_GUIBASE_CONNECTION_SET_H_INCLUDED

#include <vector>
#include <boost/signals/connection.hpp>
#include "exportdecl.h"

namespace cnoid {
    
    class CNOID_EXPORT ConnectionSet
    {
    public:
      ConnectionSet();
      ConnectionSet(const ConnectionSet& org);
      ConnectionSet& operator=(const ConnectionSet& org);
      ~ConnectionSet();

      inline bool empty() {
          return boostConnections.empty();
      }
        
      void add(const boost::signals::connection& connection);
      void add(const ConnectionSet& connections);
      void block();
      void unblock();
      void disconnect();
        
    private:
        std::vector<boost::signals::connection> boostConnections;
    };
}
        
#endif

