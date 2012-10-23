/**
   @author Shin'ichiro Nakaoka
*/

#include "YamlReader.h"
#include <cerrno>
#include <stack>
#include <iostream>
#include <yaml.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool debugTrace = false;
}

namespace cnoid {

    class YamlReaderImpl
    {
    public:
        YamlReaderImpl();
        ~YamlReaderImpl();

        void setMappingFactory(YamlReader::MappingFactoryBase* factory);
        void clearDocuments();
        bool load(const std::string& filename);
        bool load_string(const std::string& yamlstring);
        bool parse();
        void popNode();
        void addNode(YamlNode* node);
        void onDocumentStart(yaml_event_t& event);
        void onDocumentEnd(yaml_event_t& event);
        void onMappingStart(yaml_event_t& event);
        void onMappingEnd(yaml_event_t& event);
        void onSequenceStart(yaml_event_t& event);
        void onSequenceEnd(yaml_event_t& event);
        void onScalar(yaml_event_t& event);
        void onAlias(yaml_event_t& event);

        static YamlScalar* createScalar(const yaml_event_t& event);
        
        yaml_parser_t parser;
        FILE* file;

        YamlReader::MappingFactoryBase* mappingFactory;

        vector<YamlNodePtr> documents;
        int currentDocumentIndex;

        enum State { NONE, MAPPING_KEY, MAPPING_VALUE, SEQUENCE };

        struct NodeInfo {
            YamlNodePtr node;
            string key;
        };

        stack<NodeInfo> nodeStack;

        bool isRegularMultiSequenceExpected;
        vector<int> expectedSequenceSizes;

        string errorMessage;
    };
}


YamlReader::YamlReader()
{
    impl = new YamlReaderImpl();
}


YamlReaderImpl::YamlReaderImpl()
{
    yaml_parser_initialize(&parser);
    file = 0;
    mappingFactory = new YamlReader::MappingFactory<YamlMapping>();
    currentDocumentIndex = 0;
    isRegularMultiSequenceExpected = false;
}


YamlReader::~YamlReader()
{
    delete impl;
}


YamlReaderImpl::~YamlReaderImpl()
{
    yaml_parser_delete(&parser);
    
    if(file){
        fclose(file);
        file = 0;
    }

    delete mappingFactory;
}


void YamlReader::setMappingFactory(MappingFactoryBase* factory)
{
    impl->setMappingFactory(factory);
}


void YamlReaderImpl::setMappingFactory(YamlReader::MappingFactoryBase* factory)
{
    delete mappingFactory;
    mappingFactory = factory;
}


void YamlReader::expectRegularMultiSequence()
{
    impl->isRegularMultiSequenceExpected = true;
}


void YamlReader::clearDocuments()
{
    impl->clearDocuments();
}


void YamlReaderImpl::clearDocuments()
{
    while(!nodeStack.empty()){
        nodeStack.pop();
    }
    documents.clear();
}


bool YamlReader::load(const std::string& filename)
{
    return impl->load(filename);
}


bool YamlReaderImpl::load(const std::string& filename)
{
    clearDocuments();

    if(isRegularMultiSequenceExpected){
        expectedSequenceSizes.clear();
    }
    
    currentDocumentIndex = 0;

    bool result = false;

    FILE* file = fopen(filename.c_str(), "rb");

    if(file==NULL){
        errorMessage = strerror(errno);
    } else {
        yaml_parser_set_input_file(&parser, file);
        try {
            result = parse();
        }
        catch(const YamlNode::Exception& ex){
            errorMessage = str(format("%1% at line %2%, column %3%")
                               % ex.message() % ex.line() % ex.column());
        }
        fclose(file);
    }

    return result;
}


bool YamlReader::load_string(const std::string& yamlstring)
{
    return impl->load_string(yamlstring);
}

bool YamlReaderImpl::load_string(const std::string& yamlstring)
{
    clearDocuments();

    if(isRegularMultiSequenceExpected){
        expectedSequenceSizes.clear();
    }
    
    currentDocumentIndex = 0;

    bool result = false;
    
    yaml_parser_set_input_string(&parser, (const unsigned char *)(yamlstring.c_str()), yamlstring.length());
    try {
        result = parse();
    }
    catch(const YamlNode::Exception& ex){
        errorMessage = str(format("%1% at line %2%, column %3%")
                           % ex.message() % ex.line() % ex.column());
    }

    return result;
}


bool YamlReaderImpl::parse()
{
    yaml_event_t event;
    
    bool done = false;
    
    while (!done) {

        if(!yaml_parser_parse(&parser, &event)){
            goto error;
        }

        switch(event.type){
            
        case YAML_STREAM_START_EVENT:
            break;
            
        case YAML_STREAM_END_EVENT:
            done = true;
            break;
            
        case YAML_DOCUMENT_START_EVENT:
            onDocumentStart(event);
            break;
            
        case YAML_DOCUMENT_END_EVENT:
            onDocumentEnd(event);
            break;
            
        case YAML_MAPPING_START_EVENT:
            onMappingStart(event);
            break;
            
        case YAML_MAPPING_END_EVENT:
            onMappingEnd(event);
            break;
            
        case YAML_SEQUENCE_START_EVENT:
            onSequenceStart(event);
            break;
            
        case YAML_SEQUENCE_END_EVENT:
            onSequenceEnd(event);
            break;
            
        case YAML_SCALAR_EVENT:
            onScalar(event);
            break;
            
        case YAML_ALIAS_EVENT:
            onAlias(event);
            break;
            
        default:
            break;
        }

        yaml_event_delete(&event);
    }

    return !documents.empty();

  error:
    if(debugTrace){
        cout << "error" << endl;
    }
    if(parser.error != YAML_NO_ERROR && parser.problem != NULL){
        YamlNode::Exception ex;
        ex.setPosition(parser.problem_mark.line, parser.problem_mark.column);
        ex.setMessage(parser.problem);
        throw ex;
    }
    return false;
}


void YamlReaderImpl::popNode()
{
    YamlNodePtr current = nodeStack.top().node;
    nodeStack.pop();
    if(nodeStack.empty()){
        documents.push_back(current);
    } else {
        addNode(current.get());
    }
}


void YamlReaderImpl::addNode(YamlNode* node)
{
    NodeInfo& info = nodeStack.top();
    YamlNode* parent = info.node.get();
    YamlNodeType type = parent->type();
    if(type == YAML_MAPPING){
        YamlMapping* mapping = static_cast<YamlMapping*>(parent);
        mapping->insert(info.key, node);
        info.key.clear();
    } else if(type == YAML_SEQUENCE){
        YamlSequence* sequence = static_cast<YamlSequence*>(parent);
        sequence->append(node);
    }
}


void YamlReaderImpl::onDocumentStart(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onDocumentStart()" << endl;
    }
}


void YamlReaderImpl::onDocumentEnd(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onDocumentEnd()" << endl;
    }
}


void YamlReaderImpl::onMappingStart(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onMappingStart()" << endl;
    }

    NodeInfo info;
    YamlMapping* mapping = mappingFactory->create(event.start_mark.line, event.start_mark.column);
    mapping->setFlowStyle(event.data.mapping_start.style == YAML_FLOW_MAPPING_STYLE);
    info.node = mapping;
    
    nodeStack.push(info);
}


void YamlReaderImpl::onMappingEnd(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onMappingEnd()" << endl;
    }

    popNode();
}


void YamlReaderImpl::onSequenceStart(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onSequenceStart()" << endl;
    }

    NodeInfo info;
    YamlSequence* sequence;

    if(!isRegularMultiSequenceExpected){
        sequence = new YamlSequence(event.start_mark.line, event.start_mark.column);
    } else {
        size_t level = nodeStack.size();
        if(expectedSequenceSizes.size() <= level){
            expectedSequenceSizes.resize(level + 1, 0);
        }
        const int prevSize = expectedSequenceSizes[level];
        sequence = new YamlSequence(event.start_mark.line, event.start_mark.column, prevSize);
    }
    
    sequence->setFlowStyle(event.data.sequence_start.style == YAML_FLOW_SEQUENCE_STYLE);
    info.node = sequence;
    nodeStack.push(info);
}


void YamlReaderImpl::onSequenceEnd(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onSequenceEnd()" << endl;
    }

    if(isRegularMultiSequenceExpected){
        YamlSequence* sequence = static_cast<YamlSequence*>(nodeStack.top().node.get());
        const int level = nodeStack.size() - 1;
        expectedSequenceSizes[level] = sequence->size();
    }
    
    popNode();
}


void YamlReaderImpl::onScalar(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onScalar()" << endl;
    }

    yaml_char_t* value = event.data.scalar.value;
    size_t length = event.data.scalar.length;

    if(nodeStack.empty()){
        YamlNode::SyntaxException ex;
        ex.setMessage("Scalar value cannot be put on the top-level text position");
        const yaml_mark_t& start_mark = event.start_mark;
        ex.setPosition(start_mark.line, start_mark.column);
        throw ex;
    }

    NodeInfo& info = nodeStack.top();
    YamlNodePtr& parent = info.node;
    YamlNodeType type = parent->type();
     
    if(type == YAML_MAPPING){
        if(info.key.empty()){
            info.key = string((char*)value, length);
            if(info.key.empty()){
                YamlNode::SyntaxException ex;
                ex.setMessage("empty key");
                const yaml_mark_t& start_mark = event.start_mark;
                ex.setPosition(start_mark.line, start_mark.column);
                throw ex;
            }
        } else {
            YamlScalar* scalar = createScalar(event);
            addNode(scalar);
        }
    } else if(type == YAML_SEQUENCE){
        YamlScalar* scalar = createScalar(event);
        addNode(scalar);
    }
}


YamlScalar* YamlReaderImpl::createScalar(const yaml_event_t& event)
{
    YamlScalar* scalar = new YamlScalar((char*)event.data.scalar.value, event.data.scalar.length);

    const yaml_mark_t& start_mark = event.start_mark;
    scalar->line_ = start_mark.line;
    scalar->column_ = start_mark.column;

    switch(event.data.scalar.style){
    case YAML_PLAIN_SCALAR_STYLE:
        scalar->stringStyle = YAML_PLAIN_STRING;
        break;
    case YAML_SINGLE_QUOTED_SCALAR_STYLE:
        scalar->stringStyle = YAML_SINGLE_QUOTED;
        break;
    case YAML_DOUBLE_QUOTED_SCALAR_STYLE:
        scalar->stringStyle = YAML_DOUBLE_QUOTED;
        break;
    case YAML_LITERAL_SCALAR_STYLE:
        scalar->stringStyle = YAML_LITERAL;
        break;
    case YAML_FOLDED_SCALAR_STYLE:
        scalar->stringStyle = YAML_FOLDED;
        break;
    default:
        scalar->stringStyle = YAML_DOUBLE_QUOTED;
    }

    return scalar;
}


void YamlReaderImpl::onAlias(yaml_event_t& event)
{
    if(debugTrace){
        cout << "YamlReaderImpl::onAlias()" << endl;
    }
}


int YamlReader::numDocuments()
{
    return impl->documents.size();
}


YamlNode* YamlReader::document(int index)
{
    if(index >= static_cast<int>(impl->documents.size())){
        YamlNode::DocumentNotFoundException ex;
        if(index == 0){
            ex.setMessage("The yaml file does not contains any documents.");
        } else {
            ex.setMessage(str(format("The yaml file does not contains %1%-th document.") % index));
        }
        ex.setPosition(-1, -1);
        throw ex;
    }
    
    return impl->documents[index].get();
}


const std::string& YamlReader::errorMessage()
{
    return impl->errorMessage;
}
