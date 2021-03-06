/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_EFFECTS_H
#define CNOID_UTIL_SCENE_EFFECTS_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SgFog : public SgPreprocessed
{
public:
    SgFog();
    SgFog(const SgFog& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const;
    virtual void accept(SceneVisitor& visitor);

    const Vector3f& color() const { return color_; }
    template<typename Derived> void setColor(const Eigen::MatrixBase<Derived>& c) {
        color_ = c.template cast<Vector3f::Scalar>(); }
    void setVisibilityRange(float r) { visibilityRange_ = r; }
    float visibilityRange() const { return visibilityRange_; }
    
private:
    Vector3f color_;
    float  visibilityRange_;
    //int fogType;
};
typedef ref_ptr<SgFog> SgFogPtr;


class CNOID_EXPORT SgOutlineGroup : public SgGroup
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SgOutlineGroup();

    virtual void accept(SceneVisitor& visitor);
    const Vector4f& color() const { return color_; }
    void setColor(const Vector4f& color) { color_ = color; }
    void setLineWidth(float width) { lineWidth_ = width; }
    float lineWidth() const { return lineWidth_; }

private:
    Vector4f color_;
    float lineWidth_;
};
typedef ref_ptr<SgOutlineGroup> SgOutlineGroupPtr;


}

#endif
