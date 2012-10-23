/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_SIGNAL_PROXY_H_INCLUDED
#define CNOID_GUIBASE_SIGNAL_PROXY_H_INCLUDED

#include <boost/signals.hpp>

namespace cnoid {

    template <class SignalType>
    class SignalProxy
    {
      public:
        inline SignalProxy() : signal(0) { }
        inline SignalProxy(SignalType& signal) : signal(&signal) { }
        inline SignalProxy(const SignalProxy& org) : signal(org.signal) { }

        inline boost::signals::connection connect(typename SignalType::slot_function_type f){
            if(signal){
                return signal->connect(f);
            } else {
                return boost::signals::connection();
            }
        };

      private:
        SignalProxy& operator=(const SignalProxy& rhs) { } // disabled
        SignalType* signal;
    };

}

#endif
