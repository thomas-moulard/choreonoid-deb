/**
   @author Shin'ichiro Nakaoka
*/

#include "YamlNodes.h"
#include <stack>
#include <iostream>
#include <yaml.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#ifdef _WIN32
#define snprintf _snprintf_s
#endif

using namespace std;
using namespace boost;
using namespace cnoid;


namespace {
    const bool debugTrace = false;
    const char* typeNames[] = { "unknown node", "mapping", "sequence", "scalar" };
    map<string, bool> booleanSymbols;

    YamlNodePtr invalidNode;
    YamlMappingPtr invalidMapping;
    YamlSequencePtr invalidSequence;
}


YamlNode::Exception::~Exception()
{

}


// disabled
YamlNode::YamlNode(const YamlNode&)
{
    // throw an exception here ?
}


// disabled
YamlNode& YamlNode::operator=(const YamlNode&)
{
    // throw an exception here ?
    return *this;
}


void YamlNode::initialize()
{
    static bool initialized = false;

    if(!initialized){

        invalidNode = new YamlNode(YAML_NONE);

        invalidMapping = new YamlMapping();
        invalidMapping->type_ = YAML_NONE;

        invalidSequence = new YamlSequence();
        invalidSequence->type_ = YAML_NONE;
        
        booleanSymbols["true"] = true;
        booleanSymbols["yes"] = true;
        booleanSymbols["on"] = true;
        booleanSymbols["false"] = false;
        booleanSymbols["no"] = false;
        booleanSymbols["off"] = false;

        initialized = true;
    }
}


bool YamlNode::read(int &out_value) const
{
    if(type_ == YAML_SCALAR){
        const char* nptr = &(static_cast<const YamlScalar* const>(this)->stringValue[0]);
        char* endptr;
        out_value = strtol(nptr, &endptr, 10);
        if(endptr > nptr){
            return true;
        }
    }
    return false;
}


int YamlNode::toInt() const
{
    if(type_ != YAML_SCALAR){
        throwNotScalrException();
    }

    const YamlScalar* const scalar = static_cast<const YamlScalar* const>(this);
    
    const char* nptr = &(scalar->stringValue[0]);
    char* endptr;
    const int value = strtol(nptr, &endptr, 10);

    if(endptr == nptr){
        ScalarTypeMismatchException ex;
        ex.setMessage(str(format("\"%1%\" at line %2%, column %3% should be an integer value.")
                      % scalar->stringValue % line() % column()));
        ex.setPosition(line(), column());
        throw ex;
    }

    return value;
}


double YamlNode::toDouble() const
{
    if(type_ != YAML_SCALAR){
        throwNotScalrException();
    }

    const YamlScalar* const scalar = static_cast<const YamlScalar* const>(this);

    const char* nptr = &(scalar->stringValue[0]);
    char* endptr;
    const double value = strtod(nptr, &endptr);

    if(endptr == nptr){
        ScalarTypeMismatchException ex;
        ex.setMessage(str(format("\"%1%\" at line %2%, column %3% should be a double value.")
                          % scalar->stringValue % line() % column()));
        ex.setPosition(line(), column());
        throw ex;
    }

    return value;
}


bool YamlNode::toBool() const
{
    if(type_ != YAML_SCALAR){
        throwNotScalrException();
    }

    const YamlScalar* const scalar = static_cast<const YamlScalar* const>(this);

    map<string, bool>::iterator p = booleanSymbols.find(scalar->stringValue);
    if(p != booleanSymbols.end()){
        return p->second;
    }
    
    ScalarTypeMismatchException ex;
    ex.setMessage(str(format("\"%1%\" at line %2%, column %3% should be a bool value.")
                      % scalar->stringValue % line() % column()));
    ex.setPosition(line(), column());
    throw ex;
}


#ifdef _WIN32
const std::string YamlNode::toString() const
{
    if(type_ != YAML_SCALAR){
        throwNotScalrException();
    }
    return fromUtf8(static_cast<const YamlScalar* const>(this)->stringValue);
}

const std::string YamlNode::toUtf8String() const
{
    if(type_ != YAML_SCALAR){
        throwNotScalrException();
    }
    return static_cast<const YamlScalar* const>(this)->stringValue;
}    

#else

const std::string& YamlNode::toString() const
{
    if(type_ != YAML_SCALAR){
        throwNotScalrException();
    }
    return static_cast<const YamlScalar* const>(this)->stringValue;
}


const std::string& YamlNode::toUtf8String() const
{
    return toString();
}
#endif


YamlScalar::YamlScalar(const std::string& value, YamlStringStyle stringStyle)
    : stringValue(value),
      stringStyle(stringStyle)
{
    type_ = YAML_SCALAR;
    line_ = -1;
    column_ = -1;
}


YamlScalar::YamlScalar(const char* text, size_t length)
    : stringValue(text, length)
{
    type_ = YAML_SCALAR;
    stringStyle = YAML_PLAIN_STRING;
}


YamlScalar::YamlScalar(const char* text, size_t length, YamlStringStyle stringStyle)
    : stringValue(text, length),
      stringStyle(stringStyle)
{
    type_ = YAML_SCALAR;
    line_ = -1;
    column_ = -1;
}


const YamlMapping* YamlNode::toMapping() const
{
    if(type_ != YAML_MAPPING){
        throwNotMappingException();
    }
    return static_cast<const YamlMapping*>(this);
}


YamlMapping* YamlNode::toMapping()
{
    if(type_ != YAML_MAPPING){
        throwNotMappingException();
    }
    return static_cast<YamlMapping*>(this);
}


const YamlSequence* YamlNode::toSequence() const
{
    if(type_ != YAML_SEQUENCE){
        throwNotSequenceException();
    }
    return static_cast<const YamlSequence*>(this);
}


YamlSequence* YamlNode::toSequence()
{
    if(type_ != YAML_SEQUENCE){
        throwNotSequenceException();
    }
    return static_cast<YamlSequence*>(this);
}


void YamlNode::throwNotScalrException() const
{
    NotScalarException ex;
    if(hasLineInfo()){
        ex.setMessage(str(format("The %1% at line %2%, column %3% should be a scalar value.")
                      % typeNames[type_] % line() % column()));
    } else {
        ex.setMessage("Scalar value cannot be obtained from a non-scalar type yaml node.");
    }
    ex.setPosition(line(), column());
    throw ex;
}


void YamlNode::throwNotMappingException() const
{
    NotMappingException ex;
    ex.setPosition(line(), column());
    throw ex;
}


void YamlNode::throwNotSequenceException() const
{
    NotSequenceException ex;
    ex.setPosition(line(), column());
    throw ex;
}


YamlCollection::YamlCollection()
{
    isFlowStyle_ = false;
    static const char* defaultFormat = "%.6g";
    doubleFormat_ = defaultFormat;
}

    
YamlCollection::~YamlCollection()
{

}


void YamlCollection::setDoubleFormat(const char* format)
{
    doubleFormat_ = format;
}


YamlMapping::YamlMapping()
{
    type_ = YAML_MAPPING;
    line_ = -1;
    column_ = -1;
    mode = READ_MODE;
    indexCounter = 0;
    keyQuoteStyle = YAML_PLAIN_STRING;
}


YamlMapping::YamlMapping(int line, int column)
{
    type_ = YAML_MAPPING;
    line_ = line;
    column_ = column;
    mode = READ_MODE;
    indexCounter = 0;
}


YamlMapping::~YamlMapping()
{
    clear();
}


void YamlMapping::clear()
{
    values.clear();
    indexCounter = 0;
}


void YamlMapping::setKeyQuoteStyle(YamlStringStyle style)
{
    keyQuoteStyle = style;
}


YamlNode* YamlMapping::find(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUtf8(key));
    if(p != values.end()){
        return p->second.get();
    } else {
        return invalidNode.get();
    }
}


YamlMapping* YamlMapping::findMapping(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUtf8(key));
    if(p != values.end()){
        YamlNode* node = p->second.get();
        if(node->type() == YAML_MAPPING){
            return static_cast<YamlMapping*>(node);
        }
    }
    return invalidMapping.get();
}


YamlSequence* YamlMapping::findSequence(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUtf8(key));
    if(p != values.end()){
        YamlNode* node = p->second.get();
        if(node->type() == YAML_SEQUENCE){
            return static_cast<YamlSequence*>(node);
        }
    }
    return invalidSequence.get();
}


YamlNode& YamlMapping::get(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(toUtf8(key));
    if(p == values.end()){
        throwKeyNotFoundException(key);
    }
    return *p->second;
}


void YamlMapping::throwKeyNotFoundException(const std::string& key) const
{
    KeyNotFoundException ex;
    ex.setMessage(str(format("Key \"%1%\" is not found in the mapping that begins at line %2%, column %3%.")
                      % key % line() % column()));
    ex.setPosition(line(), column());
    ex.setKey(key);
    throw ex;
}


inline void YamlMapping::insertSub(const std::string& key, YamlNode* node)
{
    values.insert(make_pair(key, node));
    node->indexInMapping = indexCounter++;
}


void YamlMapping::insert(const std::string& key, YamlNodePtr node)
{
    if(!isValid()){
        throwNotMappingException();
    }
    const string uKey(toUtf8(key));
    iterator p = values.find(uKey);
    if(p != values.end()){
        values.erase(p);
    }
    insertSub(uKey, node.get());
}


YamlMapping* YamlMapping::openMapping(const std::string& key, bool doOverwrite)
{
    if(!isValid()){
        throwNotMappingException();
    }

    YamlMapping* mapping = 0;
    const string uKey(toUtf8(key));
    iterator p = values.find(uKey);
    if(p != values.end()){
        YamlNode* node = p->second.get();
        if(node->type() != YAML_MAPPING){
            values.erase(p);
        } else {
            mapping = static_cast<YamlMapping*>(node);
            if(doOverwrite){
                mapping->clear();
            }
            mapping->indexInMapping = indexCounter++;
        }
    }

    if(!mapping){
        mapping = new YamlMapping();
        mapping->doubleFormat_ = doubleFormat_;
        insertSub(uKey, mapping);
    }

    return mapping;
}


YamlMapping* YamlMapping::openFlowStyleMapping(const std::string& key, bool doOverwrite)
{
    YamlMapping* m = openMapping(key, doOverwrite);
    m->setFlowStyle(true);
    return m;
}


YamlSequence* YamlMapping::openSequence(const std::string& key, bool doOverwrite)
{
    if(!isValid()){
        throwNotMappingException();
    }

    YamlSequence* sequence = 0;
    const string uKey(toUtf8(key));
    iterator p = values.find(uKey);
    if(p != values.end()){
        YamlNode* node = p->second.get();
        if(node->type() != YAML_SEQUENCE){
            values.erase(p);
        } else {
            sequence = static_cast<YamlSequence*>(node);
            if(doOverwrite){
                sequence->clear();
            }
            sequence->indexInMapping = indexCounter++;
        }
    }

    if(!sequence){
        sequence = new YamlSequence();
        sequence->doubleFormat_ = doubleFormat_;
        insertSub(uKey, sequence);
    }

    return sequence;
}


YamlSequence* YamlMapping::openFlowStyleSequence(const std::string& key, bool doOverwrite)
{
    YamlSequence* s = openSequence(key, doOverwrite);
    s->setFlowStyle(true);
    return s;
}


bool YamlMapping::read(const std::string &key, std::string &out_value) const
{
    YamlNode* node = find(toUtf8(key));
    if(node->isValid()){
        out_value = node->toString();
        return true;
    }
    return false;
}


bool YamlMapping::readUtf8(const std::string &key, std::string &out_value) const
{
    YamlNode* node = find(toUtf8(key));
    if(node->isValid()){
        out_value = node->toUtf8String();
        return true;
    }
    return false;
}


bool YamlMapping::read(const std::string &key, bool &out_value) const
{
    YamlNode* node = find(toUtf8(key));
    if(node->isValid()){
        out_value = node->toBool();
        return true;
    }
    return false;
}


bool YamlMapping::read(const std::string &key, int &out_value) const
{
    YamlNode* node = find(toUtf8(key));
    if(node->isValid()){
        out_value = node->toInt();
        return true;
    }
    return false;
}


bool YamlMapping::read(const std::string &key, double &out_value) const
{
    YamlNode* node = find(toUtf8(key));
    if(node->isValid()){
        out_value = node->toDouble();
        return true;
    }
    return false;
}


void YamlMapping::writeUtf8(const std::string &key, const std::string& value, YamlStringStyle stringStyle)
{
    string uKey(toUtf8(key));
    iterator p = values.find(uKey);
    if(p == values.end()){
        insertSub(uKey, new YamlScalar(value, stringStyle));
    } else {
        YamlNode* node = p->second.get();
        if(node->type() == YAML_SCALAR){
            YamlScalar* scalar = static_cast<YamlScalar*>(node);
            scalar->stringValue = value;
            scalar->stringStyle = stringStyle;
            scalar->indexInMapping = indexCounter++;
        } else {
            throwNotScalrException();
        }
    }
}


/**
   This is for internal use. Text are not converted to UTF-8.
*/
void YamlMapping::writeSub(const std::string &key, const char* text, size_t length, YamlStringStyle stringStyle)
{
    const string uKey(toUtf8(key));
    iterator p = values.find(uKey);
    if(p == values.end()){
        insertSub(uKey, new YamlScalar(text, length, stringStyle));
    } else {
        YamlNode* node = p->second.get();
        if(node->type() == YAML_SCALAR){
            YamlScalar* scalar = static_cast<YamlScalar*>(node);
            scalar->stringValue = string(text, length);
            scalar->stringStyle = stringStyle;
            scalar->indexInMapping = indexCounter++;
        } else {
            throwNotScalrException();
        }
    }
}


void YamlMapping::write(const std::string &key, bool value)
{
    if(value){
        writeSub(key, "true", 4, YAML_PLAIN_STRING);
    } else {
        writeSub(key, "false", 5, YAML_PLAIN_STRING);
    }
}


void YamlMapping::write(const std::string &key, int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    writeSub(key, buf, n, YAML_PLAIN_STRING);
}


void YamlMapping::write(const std::string &key, double value)
{
    char buf[32];
    int n = snprintf(buf, 32, doubleFormat_, value);
    writeSub(key, buf, n, YAML_PLAIN_STRING);
}


void YamlMapping::writePath(const std::string &key, const std::string& value)
{
    write(key, filesystem::path(value).string(), YAML_DOUBLE_QUOTED);
}


bool YamlMapping::compareIters(const YamlMapping::const_iterator& it1, const YamlMapping::const_iterator& it2)
{
    return (it1->second->indexInMapping < it2->second->indexInMapping);
}


YamlSequence::YamlSequence()
{
    type_ = YAML_SEQUENCE;
    line_ = -1;
    column_ = -1;
}


YamlSequence::YamlSequence(int size)
    : values(size)
{
    type_ = YAML_SEQUENCE;
    line_ = -1;
    column_ = -1;
}


YamlSequence::YamlSequence(int line, int column)
{
    type_ = YAML_SEQUENCE;
    line_ = line;
    column_ = column;
}


YamlSequence::YamlSequence(int line, int column, int reservedSize)
    : values(reservedSize)
{
    type_ = YAML_SEQUENCE;
    line_ = line;
    column_ = column;
    values.resize(0);
}


YamlSequence::~YamlSequence()
{
    clear();
}


void YamlSequence::clear()
{
    values.clear();
}


void YamlSequence::reserve(int size)
{
    values.reserve(size);
}


void YamlSequence::insertLF(int maxColumns, int numValues)
{
    if(values.empty()){
        if(numValues > 0 && numValues > maxColumns){
            appendLF();
        }
    } else if((values.size() % maxColumns) == 0){
        appendLF();
    }
}


YamlMapping* YamlSequence::newMapping()
{
    YamlMapping* mapping = new YamlMapping();
    mapping->doubleFormat_ = doubleFormat_;
    append(mapping);
    return mapping;
}


void YamlSequence::append(int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    values.push_back(new YamlScalar(buf, n, YAML_PLAIN_STRING));
}


void YamlSequence::write(int i, int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    values[i] = new YamlScalar(buf, n, YAML_PLAIN_STRING);
}


void YamlSequence::append(size_t value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    values.push_back(new YamlScalar(buf, n, YAML_PLAIN_STRING));
}


void YamlSequence::append(double value)
{
    char buf[32];
    int n = snprintf(buf, 32, doubleFormat_, value);
    values.push_back(new YamlScalar(buf, n, YAML_PLAIN_STRING));
}


void YamlSequence::append(const std::string& value, YamlStringStyle stringStyle)
{
    values.push_back(new YamlScalar(toUtf8(value), stringStyle));
}


void YamlSequence::write(int i, const std::string& value, YamlStringStyle stringStyle)
{
    values[i] = new YamlScalar(toUtf8(value), stringStyle);
}


void YamlSequence::appendLF()
{
    values.push_back(new YamlNode(YAML_LF));
}
