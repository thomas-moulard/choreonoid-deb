/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_READER_H_INCLUDED
#define CNOID_UTIL_YAML_READER_H_INCLUDED

#include "YamlNodes.h"
#include "exportdecl.h"

namespace cnoid {

    class YamlReaderImpl;

    class CNOID_EXPORT YamlReader
    {
        class MappingFactoryBase {
        public:
            virtual YamlMapping* create(int line, int column) = 0;
        };

        template <class MappingType> class MappingFactory : public MappingFactoryBase {
        public:
            virtual YamlMapping* create(int line, int column) { return new MappingType(line, column); }
        };
        
      public:

        YamlReader();
        ~YamlReader();

        template <class TMapping> inline void setMappingClass() {
            setMappingFactory(new MappingFactory<TMapping>());
        }
        
        void expectRegularMultiSequence();

        bool load(const std::string& filename);
        bool load_string(const std::string& yamlstring);

        int numDocuments();
        YamlNode* document(int index = 0);

        void clearDocuments();

        const std::string& errorMessage();

      private:

        friend class YamlReaderImpl;
        
        YamlReaderImpl* impl;

        void setMappingFactory(MappingFactoryBase* factory);
    };
}

#endif
