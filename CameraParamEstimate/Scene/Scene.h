#ifndef SCENE_H
#define SCENE_H
#include <memory>
#include "gclnamespace.h"
#include "qobject.h"
namespace GCL {
/**
 * @brief The Scene class
 *      场景类： 在该类中托管光照、相机，并提供鼠标键盘事件的响应, 并提供一个默认的场景漫游器;
 */
class Camera;
class Manipulator;
class TrackBallManipulator;
class Scene: public QObject
{
    Q_OBJECT
public:
    Scene();
    virtual ~Scene();
    void set_back_color_mode(int t){back_color_mode_ = t;}
    /**
     * @brief init
     *      场景初始化
     */
    virtual void init();
    virtual void resize(int w, int h);
public:
    int get_width(){return width_;}
    int get_height(){return height_;}

public:  /// 渲染函数
    /**
     * @brief render
     *      默认的渲染流程（相机、光照...） 虽然是虚函数， 但一般不需要重载
     */
    virtual void render();
    /**
     * @brief renderObjects
     *      虚函数： 绘制需要渲染的对象
     */
    virtual void renderObjects() = 0;

public:/// Event Handlers: 事件处理器； 默认处理了场景漫游事件;
    virtual void timerRun();
    virtual void mouseMove(int posX, int posY);
    virtual void mousePress(int posX,int posY, unsigned int button, unsigned int modifier = NoModifier);
    virtual void mouseRelease(int posX, int posY, unsigned int button, unsigned int modifier = NoModifier);
    virtual void mouseDoubleClick(int posX, int posY, unsigned int button, unsigned int modifier = NoModifier);
    virtual void keyUp(const unsigned int& key, unsigned int modifier = NoModifier);
    virtual void keyDown(const unsigned int& key, unsigned int modifier = NoModifier);
    virtual void wheel(const float& delta);



public: /// 相机与操纵器
    Camera* getCamera();
    void enableManipulator(bool t);
    void setManipulator(const std::shared_ptr<Manipulator>& manipulator);
    Manipulator* getManipulator(){return manipulator_.get();}
public: /// 设置光照
    void setLighting(bool t);
    /**
     * @brief renderLights
     * 给定光照, 默认为两个白光源
     */
    virtual void renderLights();

    int interval() const;
    void setInterval(int interval);

//    void OnUpdate();
//signals:
//    void signal_update();

private:
    std::shared_ptr<Camera> camera_;
    std::shared_ptr<Manipulator> manipulator_;// just for
    bool has_light_;
    int interval_;  /// timerRun() 间隔时间, 用于动画; 默认为300ms;
    int width_;
    int height_;
    int back_color_mode_;
};
}
#endif // SCENE_H
