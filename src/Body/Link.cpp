/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Link.h"
#include <cnoid/SceneGraph>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;


Link::Link()
{
    index_ = -1;
    jointId_ = -1;
    parent_ = 0;
    T_.setIdentity();
    Tb_.setIdentity();
    Rs_.setIdentity();
    a_ = Vector3::UnitZ();
    jointType_ = FIXED_JOINT;
    q_ = 0.0;
    dq_ = 0.0;
    ddq_ = 0.0;
    u_ = 0.0;
    v_.setZero();
    w_.setZero();
    dv_.setZero();
    dw_.setZero();
    c_.setZero();
    wc_.setZero();
    m_ = 0.0;
    I_.setIdentity();
    Jm2_ = 0.0;
    F_ext_.setZero();
    q_upper_ = std::numeric_limits<double>::max();
    q_lower_ = -std::numeric_limits<double>::max();
    dq_upper_ = std::numeric_limits<double>::max();
    dq_lower_ = -std::numeric_limits<double>::max();
    info_ = new Mapping;
}


Link::Link(const Link& org)
    : name_(org.name_)
{
    index_ = -1; // should be set by a Body object
    jointId_ = org.jointId_;

    parent_ = 0;

    T_ = org.T_;
    Tb_ = org.Tb_;
    Rs_ = org.Rs_;
    
    a_ = org.a_;
    jointType_ = org.jointType_;

    q_ = org.q_;
    dq_ = org.dq_;
    ddq_ = org.ddq_;
    u_ = org.u_;

    v_ = org.v_;
    w_ = org.w_;
    dv_ = org.dv_;
    dw_ = org.dw_;
    
    c_ = org.c_;
    wc_ = org.wc_;
    m_ = org.m_;
    I_ = org.I_;
    Jm2_ = org.Jm2_;

    F_ext_ = org.F_ext_;

    q_upper_ = org.q_upper_;
    q_lower_ = org.q_lower_;
    dq_upper_ = org.dq_upper_;
    dq_lower_ = org.dq_lower_;

    //! \todo add the mode for doing deep copy of the following objects
    visualShape_ = org.visualShape_;
    collisionShape_ = org.collisionShape_;
    info_ = org.info_;
}


Link::~Link()
{
    LinkPtr link = child_;
    while(link){
        link->parent_ = 0;
        LinkPtr next = link->sibling_;
        link->sibling_ = 0;
        link = next;
    }
}


void Link::prependChild(Link* link)
{
    LinkPtr holder;
    if(link->parent_){
        holder = link;
        link->parent_->removeChild(link);
    }
    link->sibling_ = child_;
    child_ = link;
    link->parent_ = this;
}


void Link::appendChild(Link* link)
{
    LinkPtr holder;
    if(link->parent_){
        holder = link;
        link->parent_->removeChild(link);
    }
    if(!child_){
        child_ = link;
        link->sibling_ = 0;
    } else {
        Link* lastChild = child_;
        while(lastChild->sibling_){
            lastChild = lastChild->sibling_;
        }
        lastChild->sibling_ = link;
        link->sibling_ = 0;
    }
    link->parent_ = this;
}


/**
   A child link is removed from the link.
   If a link given by the parameter is not a child of the link, false is returned.
*/
bool Link::removeChild(Link* childToRemove)
{
    bool removed = false;

    Link* link = child_;
    Link* prevSibling = 0;
    while(link){
        if(link == childToRemove){
            childToRemove->parent_ = 0;
            childToRemove->sibling_ = 0;
            if(prevSibling){
                prevSibling->sibling_ = link->sibling_;
            } else {
                child_ = link->sibling_;
            }
            return true;
        }
        prevSibling = link;
        link = link->sibling_;
    }
    return false;
}


void Link::setName(const std::string& name)
{
    name_ = name;
}


std::string Link::jointTypeString() const
{
    switch(jointType_){
    case REVOLUTE_JOINT:    return "revolute";
    case SLIDE_JOINT:       return "prismatic";
    case FREE_JOINT:        return "free";
    case FIXED_JOINT:       return "fixed";
    case CRAWLER_JOINT:     return "crawler";
    case AGX_CRAWLER_JOINT: return "AgX crawler";
    default: return "unknown";
    }
}


void Link::setShape(SgNode* shape)
{
    visualShape_ = shape;
    collisionShape_ = shape;
}

void Link::setVisualShape(SgNode* shape)
{
    visualShape_ = shape;
}


void Link::setCollisionShape(SgNode* shape)
{
    collisionShape_ = shape;
}


void Link::resetInfo(Mapping* info)
{
    info_ = info;
}


template<> double Link::info(const std::string& key) const
{
    return info_->get(key).toDouble();
}


template<> double Link::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> void Link::setInfo(const std::string& key, const double& value)
{
    info_->write(key, value);
}
