/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_EIGEN_H_INCLUDED
#define CNOID_UTIL_YAML_EIGEN_H_INCLUDED

#include "YamlNodes.h"
#include <Eigen/Core>

namespace cnoid {

    template<typename Derived>
    bool read(const YamlMapping& mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
    {
        const YamlSequence& s = *mapping.findSequence(key);
        if(s.isValid()){
            const int nr = x.rows();
            const int nc = x.cols();
            const int n = s.size();
            int index = 0;
            if(n > 0){
                for(int i=0; i < nr; ++i){
                    for(int j=0; j < nc; ++j){
                        x(i, j) = s[index++].toDouble();
                        if(index == n){
                            break;
                        }
                    }
                }
            }
            return (index == nr * nc);
        }
        return false;
    }

    template<typename Derived>
    inline void readEx(const YamlMapping& mapping, const std::string& key, Eigen::MatrixBase<Derived>& x)
    {
        if(!read(mapping, key, x)){
            mapping.throwKeyNotFoundException(key);
        }
    }

    template<typename Derived>
    YamlSequence& write(YamlMapping& mapping, const std::string& key, const Eigen::MatrixBase<Derived>& x)
    {
        YamlSequence& s = *mapping.createFlowStyleSequence(key);
        const int nr = x.rows();
        const int nc = x.cols();
        if(nc == 1){
            for(int i=0; i < nr; ++i){
                s.append(x(i));
            }
        } else {
            for(int i=0; i < nr; ++i){
                s.appendLF();
                for(int j=0; j < nc; ++j){
                    s.append(x(i, j));
                }
            }
        }
        return s;
    }

}

#endif
