#ifndef GCL_MANIPULATOR_H
#define GCL_MANIPULATOR_H
#include "EventHandler.h"
#include "../../Math/MathDefines.h"
#include <memory>
namespace GCL {
/**
 *  场景漫游器类: 响应交互来改变相机位置与角度， 以观察场景;
 */
class Camera;
class  Manipulator : public EventHandler
{
public:
    Manipulator(Camera* camera);
    virtual ~Manipulator();
protected:
    Camera* getCamera() {return camera_;}
private:
    Camera* camera_;
};
}
#endif // MANIPULATOR_H
