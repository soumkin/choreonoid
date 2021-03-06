/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/OpenHRPControllerBase>
#include <cnoid/SimpleController>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/FileUtil>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;


class SimpleControllerWrapper : public SimpleControllerIO, public OpenHRPControllerBase
{
    SimpleController* controller;
    BodyPtr ioBody;
    bool ready;
    OpenHRP::DblSequence u;
    OpenHRP::DblSequence_var q;
        
public:
    
    SimpleControllerWrapper(const char* charname)
        : OpenHRPControllerBase(charname) {
        
        ready = false;

        controller = createSimpleController();
        
        if(controller){
            string modelfile = getNativePathString(
                boost::filesystem::path(shareDirectory()) / "model" / MODELFILE);
            
            BodyLoader loader;
            loader.setMessageSink(cout);
            loader.setShapeLoadingEnabled(false);
            ioBody = loader.load(modelfile);
            if(ioBody){
                ready = true;
            } else {
                cout << modelfile << " cannot be loaded." << endl;
            }
        }
    }

    ~SimpleControllerWrapper(){
        if(controller){
            delete controller;
        }
    }
    
    virtual void start(){
        if(ready){
            input();
            if(!controller->initialize(this) || !controller->start()){
                ready = false;
                cout << "Intializing the controller failed." << endl;
            }
        }
    }

    virtual void input(){
        if(ready){
            dynamicsSimulator->getCharacterAllLinkData(
                characterName.c_str(), DynamicsSimulator::JOINT_VALUE, q);
            int n = std::min((int)q->length(), ioBody->numJoints());
            for(int i=0; i < n; ++i){
                ioBody->joint(i)->q() = q[i];
            }
        }
    }

    virtual void control(){
        if(ready){
            controller->control();
        }
    }

    virtual void output() {
        if(ready){
            u.length(ioBody->numJoints());
            for(int i=0; i < ioBody->numJoints(); ++i){
                u[i] = ioBody->joint(i)->u();
            }
            dynamicsSimulator->setCharacterAllLinkData(
                characterName.c_str(), DynamicsSimulator::JOINT_TORQUE, u);
        }
    }

    virtual std::string optionString() const { return ""; }
    virtual std::vector<std::string> options() const { return std::vector<std::string>(); }
    virtual Body* body() { return ioBody; }
    virtual double timeStep() const { return OpenHRPControllerBase::timeStep; }
    virtual std::ostream& os() const { return std::cout; }
    virtual void setJointOutput(int stateTypes) { }
    virtual void setLinkOutput(Link* link, int stateTypes) { }
    virtual void setJointInput(int stateTypes) { }
    virtual void setLinkInput(Link* link, int stateTypes) { }
};


Controller_ptr OpenHRPControllerFactory_impl::create(const char* charname)
{
    OpenHRPControllerBase* controllerImpl = 0;
    try{
        controllerImpl = new SimpleControllerWrapper(charname);
        cout << "Controller for " << charname << " created." << endl;

    } catch(CORBA::SystemException& ex) {
        cerr << ex._rep_id() << endl;
        cerr << "exception in createController" << endl;
    }
    return controllerImpl->_this();
}


int main(int argc, char* argv[])
{
    OpenHRPControllerFactory_impl::run(argc, argv);
}
