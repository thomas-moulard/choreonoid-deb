/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_REFERENCED_H_INCLUDED
#define CNOID_UTIL_REFERENCED_H_INCLUDED

#include <boost/intrusive_ptr.hpp>

namespace cnoid {
    class Referenced;
}

namespace boost{
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
        friend void boost::intrusive_ptr_add_ref(Referenced* obj);
        friend void boost::intrusive_ptr_release(Referenced* obj);

        int refCounter_;
    };

    typedef boost::intrusive_ptr<Referenced> ReferencedPtr;
}

namespace boost
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
