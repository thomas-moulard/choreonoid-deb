/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_NODES_H_INCLUDED
#define CNOID_UTIL_YAML_NODES_H_INCLUDED

#include "Utf8.h"
#include <map>
#include <vector>
#include <cstring>
#include <iosfwd>
#include <boost/intrusive_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {
    class YamlNode;
}

namespace cnoid {
    void intrusive_ptr_add_ref(cnoid::YamlNode* obj);
    void intrusive_ptr_release(cnoid::YamlNode* obj);
}


namespace cnoid {

    enum YamlNodeType { YAML_NONE = 0, YAML_MAPPING, YAML_SEQUENCE, YAML_SCALAR, YAML_LF };
    enum YamlStringStyle { YAML_PLAIN_STRING, YAML_SINGLE_QUOTED, YAML_DOUBLE_QUOTED, YAML_LITERAL, YAML_FOLDED };

    class YamlScalar;
    class YamlMapping;
    class YamlSequence;
    class YamlReaderImpl;
    class YamlWriter;

    class CNOID_EXPORT YamlNode
    {
      public:

        static void initialize();

        inline bool isValid() const { return type_ != YAML_NONE; }

        inline YamlNodeType type() const { return type_; }

        int toInt() const;
        double toDouble() const;
        bool toBool() const;

        inline bool isString() const { return type_ == YAML_SCALAR; }

#ifdef _WIN32
        const std::string toString() const;
        const std::string toUtf8String() const;

        inline operator std::string () const {
            return toString();
        }
#else
        const std::string& toString() const;
        const std::string& toUtf8String() const;

        inline operator const std::string& () const {
            return toString();
        }
#endif

        inline bool isMapping() const { return type_ == YAML_MAPPING; }
        const YamlMapping* toMapping() const;
        YamlMapping* toMapping();

        inline bool isSequence() const { return type_ == YAML_SEQUENCE; }
        const YamlSequence* toSequence() const;
        YamlSequence* toSequence();

        //bool read(std::string &out_value) const;
        //bool read(bool &out_value) const;
        bool read(int &out_value) const;
        //bool read(double &out_value) const;

        inline bool hasLineInfo() const { return (line_ >= 0); }
        inline int line() const { return line_ + 1; }
        inline int column() const { return column_ + 1; }

        class CNOID_EXPORT Exception {
        public:
            virtual ~Exception();
            int line() const { return line_; }
            int column() const { return column_; }
            const std::string& message() const { return message_; }
            void setPosition(int line, int column) {
                line_ = line;
                column_ = column;
            }
            void setMessage(const std::string& m){
                message_ = m;
            }
          private:
            int line_;
            int column_;
            std::string message_;
        };
        
        class KeyNotFoundException : public Exception {
        public:
            const std::string& key() { return key_; }
            void setKey(const std::string& key) { key_ = key; }
        private:
            std::string key_;
        };
        
        class NotScalarException : public Exception {
        };
        
        class ScalarTypeMismatchException : public Exception {
        };

        class NotMappingException : public Exception {
        };

        class NotSequenceException : public Exception {
        };

        class SyntaxException : public Exception {
        };

        class DocumentNotFoundException : public Exception {
        };

      private:
        
        int refCounter;
        
      protected:

        YamlNode() : refCounter(0) { }
        YamlNode(YamlNodeType type) : refCounter(0), type_(type), line_(-1), column_(-1) { }

        virtual ~YamlNode() { }

        void throwNotScalrException() const;
        void throwNotMappingException() const;
        void throwNotSequenceException() const;

        YamlNodeType type_;

      private:

        // disabled copy operations
        YamlNode(const YamlNode&);
        YamlNode& operator=(const YamlNode&);

        int line_;
        int column_;
        int indexInMapping; // used for YamlWriter

        friend class YamlReaderImpl;
        friend class YamlWriter;
        friend class YamlScalar;
        friend class YamlMapping;
        friend class YamlSequence;

        friend void intrusive_ptr_add_ref(YamlNode* obj);
        friend void intrusive_ptr_release(YamlNode* obj);
    };

    typedef boost::intrusive_ptr<YamlNode> YamlNodePtr;


    class CNOID_EXPORT YamlScalar : public YamlNode
    {
      private:
        YamlScalar(const char* text, size_t length);
        YamlScalar(const char* text, size_t length, YamlStringStyle stringStyle);
        YamlScalar(const std::string& value, YamlStringStyle stringStyle);

        std::string stringValue;
        YamlStringStyle stringStyle;

        friend class YamlReaderImpl;
        friend class YamlWriter;
        friend class YamlNode;
        friend class YamlMapping;
        friend class YamlSequence;
    };


    class CNOID_EXPORT YamlCollection : public YamlNode
    {
      public:
        virtual ~YamlCollection();

        inline void setFlowStyle(bool isFlowStyle = true) { isFlowStyle_ = isFlowStyle; }
        inline bool isFlowStyle() const { return isFlowStyle_; }

        void setDoubleFormat(const char* format);
        inline const char* doubleFormat() { return doubleFormat_; }

      protected:
        YamlCollection();
        const char* doubleFormat_;

      private:
        YamlCollection(const YamlCollection&);
        YamlCollection& operator=(const YamlCollection&);

        bool isFlowStyle_;
    };


    class CNOID_EXPORT YamlMapping : public YamlCollection
    {
        typedef std::map<std::string, YamlNodePtr> Container;
        
      public:

        typedef Container::iterator iterator;
        typedef Container::const_iterator const_iterator;

        YamlMapping();
        YamlMapping(int line, int column);
        virtual ~YamlMapping();

        inline bool empty() const { return values.empty(); }
        inline size_t size() const { return values.size(); }
        void clear();

        void setKeyQuoteStyle(YamlStringStyle style);

        YamlNode* find(const std::string& key) const;
        YamlMapping* findMapping(const std::string& key) const;
        YamlSequence* findSequence(const std::string& key) const;

        YamlNode& get(const std::string& key) const;

        inline YamlNode& operator[](const std::string& key) const {
            return get(key);
        }

        void insert(const std::string& key, YamlNodePtr node);

        inline YamlMapping* openMapping(const std::string& key) {
            return openMapping(key, false);
        }
        
        inline YamlMapping* openFlowStyleMapping(const std::string& key) {
            return openFlowStyleMapping(key, false);
        }

        inline YamlMapping* createMapping(const std::string& key) {
            return openMapping(key, true);
        }
        
        inline YamlMapping* createFlowStyleMapping(const std::string& key) {
            return openFlowStyleMapping(key, true);
        }

        inline YamlSequence* openSequence(const std::string& key) {
            return openSequence(key, false);
        }
        
        inline YamlSequence* openFlowStyleSequence(const std::string& key){
            return openFlowStyleSequence(key, false);
        }

        inline YamlSequence* createSequence(const std::string& key){
            return openSequence(key, true);
        }
        
        inline YamlSequence* createFlowStyleSequence(const std::string& key){
            return openFlowStyleSequence(key, true);
        }
        
        bool read(const std::string &key, std::string &out_value) const;
        bool readUtf8(const std::string &key, std::string &out_value) const;
        bool read(const std::string &key, bool &out_value) const;
        bool read(const std::string &key, int &out_value) const;
        bool read(const std::string &key, double &out_value) const;

        template <class T>
        inline T read(const std::string& key) const {
            T value;
            if(read(key, value)){
                return value;
            } else {
                throwKeyNotFoundException(key);
            }
        }

        template <class T>
        inline T get(const std::string& key, const T& defaultValue) const {
            T value;
            if(read(key, value)){
                return value;
            } else {
                return defaultValue;
            }
        }

        inline std::string get(const std::string& key, const char* defaultValue) const {
            std::string value;
            if(read(key, value)){
                return value;
            } else {
                return defaultValue;
            }
        }

        void writeUtf8(const std::string &key, const std::string& value, YamlStringStyle stringStyle = YAML_PLAIN_STRING);

        void write(const std::string &key, const std::string& value, YamlStringStyle stringStyle = YAML_PLAIN_STRING) {
            writeUtf8(key, toUtf8(value), stringStyle);
        }

        void writeUtf8(const std::string &key, const char* value, YamlStringStyle stringStyle = YAML_PLAIN_STRING){
            writeUtf8(key, std::string(value), stringStyle);
        }
        
        void write(const std::string &key, const char* value, YamlStringStyle stringStyle = YAML_PLAIN_STRING){
            write(key, std::string(value), stringStyle);
        }

        void write(const std::string &key, bool value);
        void write(const std::string &key, int value);
        void write(const std::string &key, double value);
        void writePath(const std::string &key, const std::string& value);

        typedef enum { READ_MODE, WRITE_MODE } AssignMode;

        inline void setAssignMode(AssignMode mode) { this->mode = mode; }

        template <class T>
        void assign(const std::string& key, T& io_value, const T& defaultValue){
            switch(mode){
            case READ_MODE:
                if(!read(key, io_value)){
                    io_value = defaultValue;
                }
                break;
            case WRITE_MODE:
                write(key, io_value);
                break;
            }
        }
        
        inline iterator begin() { return values.begin(); }
        inline iterator end() { return values.end(); }
        inline const_iterator begin() const { return values.begin(); }
        inline const_iterator end() const { return values.end(); }

        void throwKeyNotFoundException(const std::string& key) const;

    private:

        YamlMapping(const YamlMapping&);
        YamlMapping& operator=(const YamlMapping&);

        YamlMapping* openMapping(const std::string& key, bool doOverwrite);
        YamlMapping* openFlowStyleMapping(const std::string& key, bool doOverwrite);
        YamlSequence* openSequence(const std::string& key, bool doOverwrite);
        YamlSequence* openFlowStyleSequence(const std::string& key, bool doOverwrite);

        inline void insertSub(const std::string& key, YamlNode* node);

        void writeSub(const std::string &key, const char* text, size_t length, YamlStringStyle stringStyle);

        static bool compareIters(const YamlMapping::const_iterator& it1, const YamlMapping::const_iterator& it2);

        Container values;
        AssignMode mode;
        int indexCounter;
        YamlStringStyle keyQuoteStyle;

        friend class YamlSequence;
        friend class YamlReaderImpl;
        friend class YamlWriter;
    };

    typedef boost::intrusive_ptr<YamlMapping> YamlMappingPtr;


    /**
       @todo add 'openMapping' and 'openSequence' methods
    */
    class CNOID_EXPORT YamlSequence : public YamlCollection
    {
        typedef std::vector<YamlNodePtr> Container;

    public:

        YamlSequence();
        YamlSequence(int size);
        ~YamlSequence();
        
        typedef Container::iterator iterator;
        typedef Container::const_iterator const_iterator;

        inline bool empty() const { return values.empty(); }
        inline int size() const { return values.size(); }
        void clear();
        void reserve(int size);

        inline YamlNode& front() const {
            return *values.front();
        }

        inline YamlNode& back() const {
            return *values.back();
        }

        YamlNode* at(int i) const;

        inline YamlNode& get(int i) const {
            return *values[i];
        }

        void write(int i, int value);
        void write(int i, const std::string& value, YamlStringStyle stringStyle = YAML_PLAIN_STRING);

        bool read(int i, bool &out_value) const;
        bool read(int i, int &out_value) const;
        bool read(int i, double &out_value) const;

        inline YamlNode& operator[](int i) const {
            return *values[i];
        }

        YamlMapping* newMapping();

	inline void append(YamlNodePtr node) {
            values.push_back(node);
        }
        
        void append(int value);

        /**
            @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
            @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
                   This feature is disabled if numValues = 0.
         */
        inline void append(int value, int maxColumns, int numValues = 0) {
            insertLF(maxColumns, numValues);
            append(value);
        }

        void append(size_t value);

        /**
           @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
           @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
                  This feature is disabled if numValues = 0.
        */
        inline void append(size_t value, int maxColumns, int numValues = 0){
            insertLF(maxColumns, numValues);
            append(value);
        }
        
        void append(double value);

        /**
           @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
           @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
                  This feature is disabled if numValues = 0.
        */
        inline void append(double value, int maxColumns, int numValues = 0) {
            insertLF(maxColumns, numValues);
            append(value);
        }

        void append(const std::string& value, YamlStringStyle stringStyle = YAML_PLAIN_STRING);

        /**
           @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
           @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
                  This feature is disabled if numValues = 0.
        */
        inline void append(const std::string& value, int maxColumns, int numValues = 0, YamlStringStyle stringStyle = YAML_PLAIN_STRING){
            insertLF(maxColumns, numValues);
            append(value, stringStyle);
        }

        void appendLF();
        
        inline iterator begin() { return values.begin(); }
        inline iterator end() { return values.end(); }
        inline const_iterator begin() const { return values.begin(); }
        inline const_iterator end() const { return values.end(); };

    private:

        YamlSequence(int line, int column);
        YamlSequence(int line, int column, int reservedSize);
        
        YamlSequence(const YamlSequence&);
        YamlSequence& operator=(const YamlSequence&);

        void insertLF(int maxColumns, int numValues);
        
        Container values;

        friend class YamlMapping;
        friend class YamlReaderImpl;
        friend class YamlWriter;
    };

    typedef boost::intrusive_ptr<YamlSequence> YamlSequencePtr;
}


namespace cnoid
{
    inline void intrusive_ptr_add_ref(cnoid::YamlNode* obj){
        obj->refCounter++;
    }

    inline void intrusive_ptr_release(cnoid::YamlNode* obj){
        obj->refCounter--;
        if(obj->refCounter == 0){
            delete obj;
        }
    }
};


#endif
