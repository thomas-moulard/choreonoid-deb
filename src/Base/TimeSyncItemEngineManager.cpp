/**
   @author Shin'ichiro NAKAOKA
*/

#include "TimeSyncItemEngineManager.h"
#include "ItemTreeView.h"
#include "TimeBar.h"
#include "LazyCaller.h"
#include <vector>
#include <map>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    double currentTime;

    typedef vector<TimeSyncItemEngineFactoryFunc> FactoryArray;
    typedef map<string, FactoryArray> FactoryArrayMap;
    FactoryArrayMap allFactories;

    vector<TimeSyncItemEnginePtr> engines;
    signals::connection connectionOfSelectionOrTreeChanged;
    signals::connection connectionOfTimeChanged;

    LazyCaller updateCaller;
    LazyCaller resetCaller;

    bool setTime(double time)
    {
        bool isActive = false;

        currentTime = time;

        for(size_t i=0; i < engines.size(); ++i){
            isActive |= engines[i]->onTimeChanged(time);
        }

        return isActive;
    }

    void update() {
        setTime(currentTime);
    }

    void onItemSelectionOrTreeChanged(const ItemList<Item>& selectedItems)
    {
        engines.clear();

        for(size_t i=0; i < selectedItems.size(); ++i){ 
            Item* sourceItem = selectedItems[i];
            for(FactoryArrayMap::iterator p = allFactories.begin(); p != allFactories.end(); ++p){
                FactoryArray& factories = p->second;
                for(FactoryArray::iterator q = factories.begin(); q != factories.end(); ++q){
                    TimeSyncItemEngineFactoryFunc& factoryFunc = *q;
                    TimeSyncItemEnginePtr engine = factoryFunc(sourceItem);
                    if(engine){
                        engines.push_back(engine);
                    }
                }
            }
        }

        setTime(currentTime);
    }

    void reset() {
        onItemSelectionOrTreeChanged(ItemTreeView::mainInstance()->selectedItems<Item>());
    }
}


void TimeSyncItemEngine::notifyUpdate()
{
    updateCaller.request();
}


void TimeSyncItemEngineManager::initialize()
{
    static bool initialized = false;
    
    if(!initialized){
        
        currentTime = 0.0;
        
        connectionOfSelectionOrTreeChanged =
            ItemTreeView::mainInstance()->sigSelectionOrTreeChanged().connect(onItemSelectionOrTreeChanged);
        
        connectionOfTimeChanged = TimeBar::instance()->sigTimeChanged().connect(setTime);
        
        updateCaller.set(update, IDLE_PRIORITY_LOW);

        initialized = true;
    }
}


TimeSyncItemEngineManager::TimeSyncItemEngineManager(const std::string& moduleName)
    : moduleName(moduleName)
{

}


TimeSyncItemEngineManager::~TimeSyncItemEngineManager()
{
    allFactories.erase(moduleName);
    engines.clear();
    resetCaller.request();
}


void TimeSyncItemEngineManager::addEngineFactory(TimeSyncItemEngineFactoryFunc factoryFunc)
{
    allFactories[moduleName].push_back(factoryFunc);
}
