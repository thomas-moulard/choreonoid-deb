/**
   @author Shin'ichiro Nakaoka
*/

#include "YamlWriter.h"
#include "Utf8.h"
#include <iostream>
#include <algorithm>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;


YamlWriter::YamlWriter(const std::string filename)
{
    indentWidth = 2;
    isCurrentNewLine = true;
    current = 0;
    numDocuments = 0;
    isKeyOrderPreservationMode = false;

    doubleFormat = "%.7g";

    ofs.open(filename.c_str());

    pushState(TOP, false);
}


YamlWriter::~YamlWriter()
{
    ofs.close();
}


void YamlWriter::setIndentWidth(int n)
{
    if(isTopLevel()){
        indentWidth = n;
    }
}


void YamlWriter::setKeyOrderPreservationMode(bool on)
{
    isKeyOrderPreservationMode = true;
}


bool YamlWriter::isTopLevel()
{
    return (states.size() <= 1);
}


YamlWriter::State& YamlWriter::pushState(int type, bool isFlowStyle)
{
    bool parentFlowStyle = current ? current->isFlowStyle : isFlowStyle;
    const int level = std::max(static_cast<int>(states.size() - 1), 0);
    states.push(State());
    State& state = states.top();
    state.type = type;
    state.isFlowStyle = parentFlowStyle ? true : isFlowStyle;
    state.isKeyPut = false;
    state.hasValuesBeenPut = false;
    state.indentString = string(level * indentWidth, ' ');
    current = &state;
    return state;
}


void YamlWriter::popState()
{
    states.pop();
    current = &states.top();
}


void YamlWriter::newLine()
{
    if(!isCurrentNewLine){
        ofs << "\n";
        isCurrentNewLine = true;
    }
}


void YamlWriter::indent()
{
    if(!isCurrentNewLine){
        newLine();
    }
    ofs << current->indentString;
}


void YamlWriter::startDocument()
{
    newLine();
    if(numDocuments > 0){
        ofs << "\n";
    }
    ofs << "---\n";
    
    ++numDocuments;
}


void YamlWriter::putComment(const std::string& comment, bool doNewLine)
{
    if(doNewLine){
        indent();
    }
    ofs << "# " << toUtf8(comment);
    isCurrentNewLine = false;
    newLine();
}


bool YamlWriter::makeValuePutReady()
{
    switch(current->type){
    case MAPPING:
        return current->isKeyPut;
    case SEQUENCE:
        if(!current->isFlowStyle){
            indent();
            ofs << "- ";
        }
        isCurrentNewLine = false;
        return true;
    default:
        return true;
    }
}


bool YamlWriter::startValuePut()
{
    if(makeValuePutReady()){
        if(current->type == SEQUENCE && current->isFlowStyle){
            if(current->hasValuesBeenPut){
                ofs << ", ";
            }
            if(doInsertLineFeed){
                newLine();
                indent();
                doInsertLineFeed = false;
                isCurrentNewLine = false;
            }
        }
        return true;
    }
    return false;
}


void YamlWriter::endValuePut()
{
    current->hasValuesBeenPut = true;
    if(current->type == MAPPING){
        current->isKeyPut = false;
    }
    if(!current->isFlowStyle){
        newLine();
    }
}


void YamlWriter::putString_(const std::string& value)
{
    if(startValuePut()){
        ofs << value;
        endValuePut();
    }
}


void YamlWriter::putString(const std::string& value)
{
    putString_(toUtf8(value));
}


void YamlWriter::putSingleQuotedString_(const std::string& value)
{
    if(startValuePut()){
        ofs << "'" << value << "'";
        endValuePut();
    }
}


void YamlWriter::putSingleQuotedString(const std::string& value)
{
    putSingleQuotedString_(toUtf8(value));
}


void YamlWriter::putDoubleQuotedString_(const std::string& value)
{
    if(startValuePut()){
        ofs << "\"" << value << "\"";
        endValuePut();
    }
}


void YamlWriter::putDoubleQuotedString(const std::string& value)
{
    putDoubleQuotedString_(toUtf8(value));
}


void YamlWriter::putBlockStyleString(const std::string& value, bool isLiteral)
{
    if(current->isFlowStyle){
        YamlNode::SyntaxException ex;
        ex.setMessage("A block-style string cannot be inserted into a flow-style container");
        throw ex;
    }
    
    if(startValuePut()){
        static char_separator<char> sep("\r", "\n");
        typedef tokenizer<char_separator<char> > Tokenizer;
        Tokenizer tokens(value, sep);

        if(isLiteral){
            ofs << "|\n";
        } else {
            ofs << ">\n";
        }
        const int level = std::max(static_cast<int>(states.size() - 1), 0);
        string indentString(level * indentWidth, ' ');
        ofs << indentString;
        bool afterLF = false;
        Tokenizer::iterator it = tokens.begin();
        while(it != tokens.end()){
            if(afterLF){
                ofs << indentString;
                afterLF = false;
            }
            if(*it == "\n"){
                if(++it == tokens.end()){
                    break;
                }
                ofs << "\n";
                afterLF = true;
            } else {
                ofs << toUtf8(*it++);
            }
        }
        
        endValuePut();
    }
}


void YamlWriter::putScalar(const double& value)
{
    char buf[20];
#ifdef _WIN32
    _snprintf(buf, 20, doubleFormat, value);
#else
    snprintf(buf, 20, doubleFormat, value);
#endif
    putString_(buf);
}


void YamlWriter::setDoubleFormat(const char* format)
{
    doubleFormat = format;
}


void YamlWriter::startMapping()
{
    startMappingSub(false);
}


void YamlWriter::startFlowStyleMapping()
{
    startMappingSub(true);
}


void YamlWriter::startMappingSub(bool isFlowStyle)
{
    if(startValuePut()){
        int parentType = current->type;
        State& state = pushState(MAPPING, isFlowStyle);
        if(!state.isFlowStyle){
            if(parentType == MAPPING){
                newLine();
            }
        } else {
            ofs << "{ ";
            isCurrentNewLine = false;
        }
    }
}


void YamlWriter::putKey_(const std::string& key, YamlStringStyle style)
{
    if(current->type == MAPPING && !current->isKeyPut){
        if(current->isFlowStyle){
            if(current->hasValuesBeenPut){
                ofs << ", ";
            }
        } else {
            indent();
        }

        switch(style){
        case YAML_SINGLE_QUOTED:
            ofs << "'" << key << "': ";
            break;
        case YAML_DOUBLE_QUOTED:
            ofs << "\"" << key << "\": ";
            break;
        default:
            ofs << key << ": ";
            break;
        }

        current->isKeyPut = true;
        isCurrentNewLine = false;
    }
}


void YamlWriter::putKey(const std::string& key, YamlStringStyle style)
{
    putKey_(toUtf8(key), style);
}


void YamlWriter::endMapping()
{
    if(current->type == MAPPING){
        if(current->isFlowStyle){
            ofs << " }";
        }
        popState();
        endValuePut();
    }
}


void YamlWriter::startSequence()
{
    startSequenceSub(false);
}


void YamlWriter::startFlowStyleSequence()
{
    startSequenceSub(true);
}


void YamlWriter::startSequenceSub(bool isFlowStyle)
{
    if(startValuePut()){
        State& state = pushState(SEQUENCE, isFlowStyle);
        if(!state.isFlowStyle){
            if(!isTopLevel()){
                newLine();
            }
        } else {
            ofs << "[ ";
            isCurrentNewLine = false;
            doInsertLineFeed = false;
        }
    }
}


void YamlWriter::endSequence()
{
    if(current->type == SEQUENCE){
        if(current->isFlowStyle){
            ofs << " ]";
        }
        popState();
        endValuePut();
    }
}


void YamlWriter::putNode(YamlNode& node)
{
    YamlNodePtr pNode = &node;
    putNode(pNode);
}


void YamlWriter::putNode(const YamlNodePtr& node)
{
    switch(node->type()){

    case YAML_SCALAR: {
        const YamlScalar* scalar = static_cast<const YamlScalar*>(node.get());
        if(scalar->stringStyle == YAML_PLAIN_STRING){
            putString_(scalar->stringValue);
        } else if(scalar->stringStyle == YAML_SINGLE_QUOTED){
            putSingleQuotedString_(scalar->stringValue);
        } else if(scalar->stringStyle == YAML_DOUBLE_QUOTED){
            putDoubleQuotedString_(scalar->stringValue);
        } else if(scalar->stringStyle == YAML_LITERAL){
            putLiteralString(scalar->stringValue);
        } else if(scalar->stringStyle == YAML_FOLDED){
            putFoldedString(scalar->stringValue);
        } else {
            putDoubleQuotedString_(scalar->stringValue);
        }
    }
    break;

    case YAML_MAPPING:
        putMappingNode(static_cast<const YamlMapping*>(node.get()));
        break;

    case YAML_SEQUENCE:
        putSequenceNode(static_cast<const YamlSequence*>(node.get()));
        break;

    case YAML_LF:
        if(current->isFlowStyle){
            doInsertLineFeed = true;
        }
        break;

    default:
        cout << "hogehoge" << endl;
        throw "hoge";
    }
}


void YamlWriter::putMappingNode(const YamlMapping* mapping)
{
    if(mapping->isFlowStyle()){
        startFlowStyleMapping();
    } else {
        startMapping();
    }

    if(isKeyOrderPreservationMode){
        const int n(mapping->size());
        vector<YamlMapping::const_iterator> iters(n);
        int index = 0;
        for(YamlMapping::const_iterator it = mapping->begin(); it != mapping->end(); ++it){
            iters[index++] = it;
        }

        struct KeyOrderCmpFunc {
            bool operator()(const YamlMapping::const_iterator& it1, const YamlMapping::const_iterator& it2) const {
                return (it1->second->indexInMapping < it2->second->indexInMapping);
            }
        };

        std::sort(iters.begin(), iters.end(), &YamlMapping::compareIters);

        for(int i=0; i < n; ++i){
            YamlMapping::const_iterator& it = iters[i];
            putKey_(it->first, mapping->keyQuoteStyle);
            const YamlNodePtr& node = it->second;
            putNode(node);
        }        
    } else {
        for(YamlMapping::const_iterator it = mapping->begin(); it != mapping->end(); ++it){
            putKey_(it->first, mapping->keyQuoteStyle);
            const YamlNodePtr& node = it->second;
            putNode(node);
        }
    }

    endMapping();
}


void YamlWriter::putSequenceNode(const YamlSequence* sequence)
{
    if(sequence->isFlowStyle()){
        startFlowStyleSequence();
    } else {
        startSequence();
    }

    const int n = sequence->size();
    for(int i=0; i < n; ++i){
        putNode(sequence->values[i]);
    }

    endSequence();
}

        

