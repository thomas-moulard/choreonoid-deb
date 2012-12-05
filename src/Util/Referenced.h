/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_REFERENCED_H_INCLUDED
#define CNOID_UTIL_REFERENCED_H_INCLUDED

#include <boost/intrusive_ptr.hpp>

namespace cnoid {
    class Referenced;
}

namespace cnoid {
    void intrusive_ptr_add_ref(cnoid::Referenced* obj);
    void intrusive_ptr_release(cnoid::Referenced* obj);
}

namespace cnoid {
    
    class Referenced
    {
      public:
        inline Referenced() { refCounter_ = 0; }
        virtual ~Referenced() { }

    protected:
        inline int refCounter() { return refCounter_; }

      private:
        friend void intrusive_ptr_add_ref(Referenced* obj);
        friend void intrusive_ptr_release(Referenced* obj);

        int refCounter_;
    };

    typedef boost::intrusive_ptr<Referenced> ReferencedPtr;
}

namespace cnoid
{
    inline void intrusive_ptr_add_ref(cnoid::Referenced* obj){
        obj->refCounter_++;
    }

    inline void intrusive_ptr_release(cnoid::Referenced* obj){
        obj->refCounter_--;
        if(obj->refCounter_ == 0){
            delete obj;
        }
    }
};

#endif
