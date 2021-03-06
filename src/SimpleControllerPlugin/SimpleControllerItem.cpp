/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleControllerItem.h"
#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/ProjectManager>
#include <QLibrary>
#include <boost/bind.hpp>
#include <boost/dynamic_bitset.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

enum {
    INPUT_JOINT_DISPLACEMENT = 0,
    INPUT_JOINT_FORCE = 1,
    INPUT_JOINT_VELOCITY = 2,
    INPUT_JOINT_ACCELERATION = 3,
    INPUT_LINK_POSITION = 4,
    INPUT_NONE = 5
};

enum {
    OUTPUT_JOINT_FORCE = 0,
    OUTPUT_JOINT_DISPLACEMENT = 1,
    OUTPUT_JOINT_VELOCITY = 2,
    OUTPUT_JOINT_ACCELERATION = 3,
    OUTPUT_LINK_POSITION = 4,
    OUTPUT_NONE = 5
};

}

namespace cnoid {

class SimpleControllerItemImpl : public SimpleControllerIO
{
public:
    SimpleControllerItem* self;
    SimpleController* controller;
    Body* simulationBody;
    BodyPtr ioBody;
    ControllerItemIO* io;

    bool isInputStateTypeSetUpdated;
    bool isOutputStateTypeSetUpdated;
    vector<unsigned short> inputLinkIndices;
    vector<char> inputStateTypes;
    vector<unsigned short> outputLinkIndices;
    vector<char> outputStateTypes;

    ConnectionSet inputDeviceStateConnections;
    boost::dynamic_bitset<> inputDeviceStateChangeFlag;
        
    ConnectionSet outputDeviceStateConnections;
    boost::dynamic_bitset<> outputDeviceStateChangeFlag;

    vector<SimpleControllerItemPtr> childControllerItems;

    vector<char> linkIndexToInputStateTypeMap;
    vector<char> linkIndexToOutputStateTypeMap;
        
    MessageView* mv;

    std::string controllerModuleName;
    std::string controllerModuleFileName;
    QLibrary controllerModule;
    bool doReloading;
    Selection pathBase;

    enum PathBase {
        CONTROLLER_DIRECTORY = 0,
        PROJECT_DIRECTORY,
        N_PATH_BASE
    };

    SimpleControllerItemImpl(SimpleControllerItem* self);
    SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org);
    ~SimpleControllerItemImpl();
    void unloadController();
    void initializeIoBody(Body* body);
    bool initialize(ControllerItemIO* io, Body* sharedIoBody);
    void input();
    void onInputDeviceStateChanged(int deviceIndex);
    void onOutputDeviceStateChanged(int deviceIndex);
    void output();
    bool onReloadingChanged(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);

    // virtual functions of SimpleControllerIO
    virtual std::string optionString() const;
    virtual std::vector<std::string> options() const;
    virtual Body* body();
    virtual double timeStep() const;
    virtual std::ostream& os() const;
    virtual void setJointOutput(int stateTypes);
    virtual void setLinkOutput(Link* link, int stateTypes);
    virtual void setJointInput(int stateTypes);
    virtual void setLinkInput(Link* link, int stateTypes);
};

}


SimpleControllerItem::SimpleControllerItem()
{
    setName("SimpleController");
    impl = new SimpleControllerItemImpl(this);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self)
    : self(self),
      pathBase(N_PATH_BASE, CNOID_GETTEXT_DOMAIN_NAME)
{
    controller = 0;
    io = 0;
    mv = MessageView::instance();
    doReloading = true;
    pathBase.setSymbol(CONTROLLER_DIRECTORY, N_("Controller directory"));
    pathBase.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    pathBase.select(CONTROLLER_DIRECTORY);
}


SimpleControllerItem::SimpleControllerItem(const SimpleControllerItem& org)
    : ControllerItem(org)
{
    impl = new SimpleControllerItemImpl(this, *org.impl);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org)
    : self(self),
      pathBase(org.pathBase)
{
    controller = 0;
    io = 0;
    mv = MessageView::instance();
    controllerModuleName = org.controllerModuleName;
    doReloading = org.doReloading;
}


SimpleControllerItem::~SimpleControllerItem()
{
    delete impl;
}


SimpleControllerItemImpl::~SimpleControllerItemImpl()
{
    unloadController();
    inputDeviceStateConnections.disconnect();
    outputDeviceStateConnections.disconnect();
}


void SimpleControllerItemImpl::unloadController()
{
    if(controller){
        delete controller;
        controller = 0;
    }

    if(controllerModule.unload()){
        mv->putln(fmt(_("The controller module \"%2%\" of %1% has been unloaded."))
                  % self->name() % controllerModuleFileName);
    }
}


void SimpleControllerItem::onDisconnectedFromRoot()
{
    if(!isActive()){
        impl->unloadController();
    }
    impl->childControllerItems.clear();
}


Item* SimpleControllerItem::doDuplicate() const
{
    return new SimpleControllerItem(*this);
}


void SimpleControllerItem::setController(const std::string& name)
{
    impl->unloadController();
    impl->controllerModuleName = name;
    impl->controllerModuleFileName.clear();
}


void SimpleControllerItemImpl::initializeIoBody(Body* body)
{
    ioBody = body->clone();

    inputDeviceStateConnections.disconnect();
    outputDeviceStateConnections.disconnect();
    const DeviceList<>& devices = body->devices();
    const DeviceList<>& ioDevices = ioBody->devices();
    inputDeviceStateChangeFlag.resize(devices.size());
    inputDeviceStateChangeFlag.reset();
    outputDeviceStateChangeFlag.resize(ioDevices.size());
    outputDeviceStateChangeFlag.reset();
    for(size_t i=0; i < devices.size(); ++i){
        inputDeviceStateConnections.add(
            devices[i]->sigStateChanged().connect(
                boost::bind(&SimpleControllerItemImpl::onInputDeviceStateChanged, this, i)));
        outputDeviceStateConnections.add(
            ioDevices[i]->sigStateChanged().connect(
                boost::bind(&SimpleControllerItemImpl::onOutputDeviceStateChanged, this, i)));
    }
}


bool SimpleControllerItem::initialize(ControllerItemIO* io)
{
    return impl->initialize(io, 0);
}


SimpleController* SimpleControllerItem::initialize(ControllerItemIO* io, Body* sharedIoBody)
{
    if(impl->initialize(io, sharedIoBody)){
        return impl->controller;
    }
    return 0;
}
    
    
bool SimpleControllerItemImpl::initialize(ControllerItemIO* io, Body* sharedIoBody)
{
    this->io = io;
    bool result = false;

    if(!controller){

        filesystem::path dllPath(controllerModuleName);
        if(!checkAbsolute(dllPath)){
            if (pathBase.is(CONTROLLER_DIRECTORY))
            dllPath =
                filesystem::path(executableTopDirectory()) /
                CNOID_PLUGIN_SUBDIR / "simplecontroller" / dllPath;
            else {
                const string& projectFileName = ProjectManager::instance()->getProjectFileName();
                if (projectFileName.empty()){
                    mv->putln(_("Please save the project."));
                    return false;
                }else{
                    dllPath = boost::filesystem::path(projectFileName).parent_path() / dllPath;
                }
            }
        }
        controllerModuleFileName = getNativePathString(dllPath);
        controllerModule.setFileName(controllerModuleFileName.c_str());
        
        if(controllerModule.isLoaded()){
            mv->putln(fmt(_("The controller module of %1% has already been loaded.")) % self->name());
            
            // This should be called to make the reference to the DLL.
            // Otherwise, QLibrary::unload() unloads the DLL without considering this instance.
            controllerModule.load();
            
        } else {
            mv->put(fmt(_("Loading the controller module \"%2%\" of %1% ... "))
                    % self->name() % controllerModuleFileName);
            if(!controllerModule.load()){
                mv->put(_("Failed.\n"));
                mv->putln(controllerModule.errorString());
            } else {                
                mv->putln(_("OK!"));
            }
        }
        
        if(controllerModule.isLoaded()){
            SimpleController::Factory factory =
                (SimpleController::Factory)controllerModule.resolve("createSimpleController");
            if(!factory){
                mv->putln(_("The factory function \"createSimpleController()\" is not found in the controller module."));
            } else {
                controller = factory();
                if(!controller){
                    mv->putln(_("The factory failed to create a controller instance."));
                } else {
                    mv->putln(_("A controller instance has successfully been created."));
                }
            }
        }
    }

    childControllerItems.clear();

    if(controller){
        ioBody = sharedIoBody;
        if(!ioBody){
            initializeIoBody(io->body());
        }
        
        controller->setIO(this);

        isInputStateTypeSetUpdated = true;
        isOutputStateTypeSetUpdated = true;
        inputLinkIndices.clear();
        inputStateTypes.clear();
        outputLinkIndices.clear();
        outputStateTypes.clear();
        
        result = controller->initialize(this);

        // try the old API
        if(!result){
            setJointOutput(SimpleControllerIO::JOINT_TORQUE);
            setJointInput(SimpleControllerIO::JOINT_DISPLACEMENT);
            result = controller->initialize();
        }
        
        if(!result){
            mv->putln(fmt(_("%1%'s initialize method failed.")) % self->name());
            if(doReloading){
                self->stop();
            }
        } else {
            simulationBody = io->body();

            for(Item* child = self->childItem(); child; child = child->nextItem()){
                SimpleControllerItem* childControllerItem = dynamic_cast<SimpleControllerItem*>(child);
                if(childControllerItem){
                    SimpleController* childController = childControllerItem->initialize(io, ioBody);
                    if(childController){
                        childControllerItems.push_back(childControllerItem);
                    }
                }
            }
        }
    }

    return result;
}


std::string SimpleControllerItemImpl::optionString() const
{
    if(io){
        const std::string& opt1 = io->optionString();
        const std::string& opt2 = self->optionString();
        if(!opt1.empty()){
            if(opt2.empty()){
                return opt1;
            } else {
                return opt1 + " " + opt2;
            }
        }
    }
    return self->optionString();
}


std::vector<std::string> SimpleControllerItemImpl::options() const
{
    vector<string> options;
    self->splitOptionString(optionString(), options);
    return options;
}


Body* SimpleControllerItemImpl::body()
{
    return ioBody;
}


double SimpleControllerItem::timeStep() const
{
    return impl->io ? impl->io->timeStep() : 0.0;
}


double SimpleControllerItemImpl::timeStep() const
{
    return io->timeStep();
}


std::ostream& SimpleControllerItemImpl::os() const
{
    return mv->cout();
}


void SimpleControllerItemImpl::setJointInput(int stateTypes)
{
    if(!stateTypes){
        linkIndexToInputStateTypeMap.clear();
    } else {
        linkIndexToInputStateTypeMap.resize(ioBody->numLinks(), 0);
        const int nj = ioBody->numJoints();
        for(int i=0; i < nj; ++i){
            int linkIndex = ioBody->joint(i)->index();
            if(linkIndex >= 0){
                linkIndexToInputStateTypeMap[linkIndex] = stateTypes;
            }
        }
    }
    isInputStateTypeSetUpdated = true;
}


void SimpleControllerItemImpl::setLinkInput(Link* link, int stateTypes)
{
    if(link->index() >= linkIndexToInputStateTypeMap.size()){
        linkIndexToInputStateTypeMap.resize(link->index() + 1, 0);
    }
    linkIndexToInputStateTypeMap[link->index()] = stateTypes;
    isInputStateTypeSetUpdated = true;
}


void SimpleControllerItemImpl::setJointOutput(int stateTypes)
{
    if(!stateTypes){
        linkIndexToOutputStateTypeMap.clear();
    } else {
        linkIndexToOutputStateTypeMap.resize(ioBody->numLinks(), 0);
        const int nj = ioBody->numJoints();
        for(int i=0; i < nj; ++i){
            int linkIndex = ioBody->joint(i)->index();
            if(linkIndex >= 0){
                linkIndexToOutputStateTypeMap[linkIndex] = stateTypes;
            }
        }
    }
    isOutputStateTypeSetUpdated = true;
}


void SimpleControllerItemImpl::setLinkOutput(Link* link, int stateTypes)
{
    if(link->index() >= linkIndexToOutputStateTypeMap.size()){
        linkIndexToOutputStateTypeMap.resize(link->index() + 1, 0);
    }
    linkIndexToOutputStateTypeMap[link->index()] = stateTypes;
    isOutputStateTypeSetUpdated = true;
}


static int getInputStateTypeIndex(int type){
    switch(type){
    case SimpleControllerIO::JOINT_DISPLACEMENT: return INPUT_JOINT_DISPLACEMENT;
    case SimpleControllerIO::JOINT_VELOCITY:     return INPUT_JOINT_VELOCITY;
    case SimpleControllerIO::JOINT_ACCELERATION: return INPUT_JOINT_ACCELERATION;
    case SimpleControllerIO::JOINT_FORCE:        return INPUT_JOINT_FORCE;
    case SimpleControllerIO::LINK_POSITION:      return INPUT_LINK_POSITION;
    default:
        return INPUT_NONE;
    }
}


static int getOutputStateTypeIndex(int type){
    switch(type){
    case SimpleControllerIO::JOINT_DISPLACEMENT: return OUTPUT_JOINT_DISPLACEMENT;
    case SimpleControllerIO::JOINT_VELOCITY:     return OUTPUT_JOINT_VELOCITY;
    case SimpleControllerIO::JOINT_ACCELERATION: return OUTPUT_JOINT_ACCELERATION;
    case SimpleControllerIO::JOINT_FORCE:        return OUTPUT_JOINT_FORCE;
    case SimpleControllerIO::LINK_POSITION:      return OUTPUT_LINK_POSITION;
    default:
        return OUTPUT_NONE;
    }
}


static void updateIOStateTypeSet
(vector<char>& linkIndexToStateTypeMap, vector<unsigned short>& linkIndices, vector<char>& stateTypes,
 boost::function<int(int type)> getStateTypeIndex)
{
    linkIndices.clear();
    stateTypes.clear();

    for(size_t i=0; i < linkIndexToStateTypeMap.size(); ++i){
        bitset<5> types(linkIndexToStateTypeMap[i]);
        if(types.any()){
            const int n = types.count();
            linkIndices.push_back(i);
            stateTypes.push_back(n);
            for(int j=0; j < 5; ++j){
                if(types.test(j)){
                    stateTypes.push_back(getStateTypeIndex(1 << j));
                }
            }
        }
    }
}


bool SimpleControllerItem::start()
{
    if(impl->controller->start()){
        for(size_t i=0; i < impl->childControllerItems.size(); ++i){
            if(!impl->childControllerItems[i]->start()){
                return false;
            }
        }
    }
    return true;
}


void SimpleControllerItem::input()
{
    impl->input();
    
    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->impl->input();
    }
}


void SimpleControllerItemImpl::input()
{
    if(isInputStateTypeSetUpdated){
        updateIOStateTypeSet(linkIndexToInputStateTypeMap, inputLinkIndices, inputStateTypes, getInputStateTypeIndex);
        isInputStateTypeSetUpdated = false;
    }

    int typeArrayIndex = 0;
    for(size_t i=0; i < inputLinkIndices.size(); ++i){
        const int linkIndex = inputLinkIndices[i];
        const Link* simLink = simulationBody->link(linkIndex);
        Link* ioLink = ioBody->link(linkIndex);
        const int n = inputStateTypes[typeArrayIndex++];
        for(int j=0; j < n; ++j){
            switch(inputStateTypes[typeArrayIndex++]){
            case INPUT_JOINT_DISPLACEMENT:
                ioLink->q() = simLink->q();
                break;
            case INPUT_JOINT_FORCE:
                ioLink->u() = simLink->u();
                break;
            case INPUT_LINK_POSITION:
                ioLink->T() = simLink->T();
                break;
            case INPUT_JOINT_VELOCITY:
                ioLink->dq() = simLink->dq();
                break;
            case INPUT_JOINT_ACCELERATION:
                ioLink->ddq() = simLink->ddq();
                break;
            default:
                break;
            }
        }
    }
                
    if(inputDeviceStateChangeFlag.any()){
        const DeviceList<>& devices = simulationBody->devices();
        const DeviceList<>& ioDevices = ioBody->devices();
        boost::dynamic_bitset<>::size_type i = inputDeviceStateChangeFlag.find_first();
        while(i != inputDeviceStateChangeFlag.npos){
            Device* ioDevice = ioDevices[i];
            ioDevice->copyStateFrom(*devices[i]);
            outputDeviceStateConnections.block(i);
            ioDevice->notifyStateChange();
            outputDeviceStateConnections.unblock(i);
            i = inputDeviceStateChangeFlag.find_next(i);
        }
        inputDeviceStateChangeFlag.reset();
    }
}


void SimpleControllerItemImpl::onInputDeviceStateChanged(int deviceIndex)
{
    inputDeviceStateChangeFlag.set(deviceIndex);
}


bool SimpleControllerItem::control()
{
    bool result = impl->controller->control();

    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        if(impl->childControllerItems[i]->impl->controller->control()){
            result = true;
        }
    }
        
    return result;
}


void SimpleControllerItemImpl::onOutputDeviceStateChanged(int deviceIndex)
{
    outputDeviceStateChangeFlag.set(deviceIndex);
}


void SimpleControllerItem::output()
{
    impl->output();
    
    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->impl->output();
    }
}


void SimpleControllerItemImpl::output()
{
    if(isOutputStateTypeSetUpdated){
        updateIOStateTypeSet(linkIndexToOutputStateTypeMap, outputLinkIndices, outputStateTypes, getOutputStateTypeIndex);
        isOutputStateTypeSetUpdated = false;
    }

    int typeArrayIndex = 0;
    for(size_t i=0; i < outputLinkIndices.size(); ++i){
        const int linkIndex = outputLinkIndices[i];
        Link* simLink = simulationBody->link(linkIndex);
        const Link* ioLink = ioBody->link(linkIndex);
        const int n = outputStateTypes[typeArrayIndex++];
        for(int j=0; j < n; ++j){
            switch(outputStateTypes[typeArrayIndex++]){
            case OUTPUT_JOINT_FORCE:
                simLink->u() = ioLink->u();
                break;
            case OUTPUT_JOINT_DISPLACEMENT:
                simLink->q() = ioLink->q();
                break;
            case OUTPUT_JOINT_VELOCITY:
                simLink->dq() = ioLink->dq();
                break;
            case OUTPUT_JOINT_ACCELERATION:
                simLink->ddq() = ioLink->ddq();
                break;
            case OUTPUT_LINK_POSITION:
                simLink->T() = ioLink->T();
                break;
            default:
                break;
            }
        }
    }

    if(outputDeviceStateChangeFlag.any()){
        const DeviceList<>& devices = simulationBody->devices();
        const DeviceList<>& ioDevices = ioBody->devices();
        boost::dynamic_bitset<>::size_type i = outputDeviceStateChangeFlag.find_first();
        while(i != outputDeviceStateChangeFlag.npos){
            Device* device = devices[i];
            device->copyStateFrom(*ioDevices[i]);
            inputDeviceStateConnections.block(i);
            device->notifyStateChange();
            inputDeviceStateConnections.unblock(i);
            i = outputDeviceStateChangeFlag.find_next(i);
        }
        outputDeviceStateChangeFlag.reset();
    }
}


void SimpleControllerItem::stop()
{
    if(impl->doReloading || !findRootItem()){
        impl->unloadController();
    }

    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->stop();
    }
    impl->childControllerItems.clear();

    impl->io = 0;
}


bool SimpleControllerItemImpl::onReloadingChanged(bool on)
{
    doReloading = on;
    return true;
}


void SimpleControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void SimpleControllerItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Relative Path Base"), pathBase, changeProperty(pathBase));

    FileDialogFilter filter;
    filter.push_back( string(_(" Dynamic Link Library ")) + DLLSFX );
    string dir;
    if(!controllerModuleName.empty() && checkAbsolute(filesystem::path(controllerModuleName)))
        dir = filesystem::path(controllerModuleName).parent_path().string();
    else{
        if(pathBase.is(CONTROLLER_DIRECTORY))
            dir = (filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "simplecontroller").string();
    }
    putProperty(_("Controller module"), FilePath(controllerModuleName, filter, dir),
                boost::bind(&SimpleControllerItem::setController, self, _1), true);
    putProperty(_("Reloading"), doReloading,
                boost::bind(&SimpleControllerItemImpl::onReloadingChanged, this, _1));
}


bool SimpleControllerItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    return impl->store(archive);
}


bool SimpleControllerItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("controller", controllerModuleName);
    archive.write("reloading", doReloading);
    archive.write("RelativePathBase", pathBase.selectedSymbol(), DOUBLE_QUOTED);
    return true;
}


bool SimpleControllerItem::restore(const Archive& archive)
{
    if(!ControllerItem::restore(archive)){
        return false;
    }
    return impl->restore(archive);
}


bool SimpleControllerItemImpl::restore(const Archive& archive)
{
    string value;
    if(archive.read("controller", value)){
        controllerModuleName = archive.expandPathVariables(value);
    }
    archive.read("reloading", doReloading);
    string symbol;
    if (archive.read("RelativePathBase", symbol)){
        pathBase.select(symbol);
    }
    return true;
}
