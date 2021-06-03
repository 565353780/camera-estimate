#ifndef TRACKBALLMANIPULATOR_H
#define TRACKBALLMANIPULATOR_H
/*
Authors: Shiwei Wang
Email(s): wangshiwei@abaci3d.com

*/
#include "Manipulator.h"
#include "../../Math/MathDefines.h"
namespace GCL {

class  TrackBallManipulator : public Manipulator
{
public:
    TrackBallManipulator(Camera* camera);
    const char* getEventHandlerName() const {return "TrackManipulator";}
    virtual ~TrackBallManipulator() {}

    Scalar speed() const;
    void setSpeed(const Scalar &speed);
    void rotate(int posX, int posY);
    void pan(int posX, int posY);
    void rotateAlongDir(double angle);
protected:
    int mouseMove(int posX, int posY);
    int mousePress(int posX,int posY, unsigned int button, unsigned int modifier = NoModifier);
    int mouseDoubleClick(int posX, int posY, unsigned int button, unsigned int modifier = NoModifier);
    int mouseRelease(int posX, int posY, unsigned int button, unsigned int modifier = NoModifier);
    int wheel(const float& delta);

private:
    void computeUpAndSideDirection(Vec3& upDirection, Vec3& sideDirection);
    int hitCenterPlane(int posX, int posY, Vec3& intersect);

    Scalar rotate_speed_;
    Scalar zoom_speed_;

private:
    int last_posX_=-1;
    int last_posY_=-1;

    bool left_press_;
};
}

#endif // TRACKBALLMANIPULATOR_H
