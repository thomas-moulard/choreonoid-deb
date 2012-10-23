/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_PUT_PROPERTY_FUNCTION_H_INCLUDED
#define CNOID_GUIBASE_PUT_PROPERTY_FUNCTION_H_INCLUDED

#include <vector>
#include <string>
#include <boost/function.hpp>

namespace cnoid {

    class Selection
    {
    public:
        inline Selection(std::vector<std::string> labels, int which = 0) : labels(labels), which(which) { }
        inline std::string label() const { return labels[which]; }
        std::vector<std::string> labels;
        int which;
    };

    class PutPropertyFunction
    {
    public:
        virtual ~PutPropertyFunction() { }

        // bool
        virtual void operator()(const std::string& name, bool value) = 0;
        virtual void operator()(const std::string& name, bool value, const boost::function<bool(bool)>& changeFunc) = 0;

        // int
        virtual void operator()(const std::string& name, int value) = 0;
        virtual void operator()(const std::string& name, int value, const boost::function<bool(int)>& changeFunc) = 0;

        // double
        virtual void operator()(const std::string& name, double value) = 0;
        virtual void operator()(const std::string& name, double value, const boost::function<bool(double)>& changeFunc) = 0;

        // string
        virtual void operator()(const std::string& name, const std::string& value) = 0;
        virtual void operator()(const std::string& name, const std::string& value,
                                const boost::function<bool(const std::string&)>& changeFunc) = 0;

        // selection
        virtual void operator()(const std::string& name, Selection selection) = 0;
        virtual void operator()(const std::string& name, Selection selection,
                                const boost::function<bool(int which)>& changeFunc) = 0;
    };
}

#endif
