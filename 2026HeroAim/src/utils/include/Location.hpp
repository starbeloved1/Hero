#pragma once
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include "Log.hpp"


/*实际的坐标转换由Solver类实现，它继承自BaseSolver：

1. 1.
   XYZ ↔ PYD转换 ：
   
   - XYZ2PYD：通过三角函数计算pitch、yaw、distance
   - PYD2XYZ：通过三角函数反向计算x、y、z
2. 2.
   CXYD ↔ XYZ转换 ：
   
   - CXYD2XYZ：根据相机内参将像素坐标转换为空间坐标
   - XYZ2CXYD：将空间坐标投影到像素坐标
3. 3.
   IMU融合 ：
   
   - camera2world：将相机坐标系下的点转换到世界坐标系（融合IMU数据）
   - world2camera：将世界坐标系下的点转换到相机坐标系*/
// 定义三种坐标类型
struct XYZ {
    double x, y, z;
    XYZ() : x(0), y(0), z(0) {}
    XYZ(double x, double y, double z) : x(x), y(y), z(z) {}
    double dist(){return std::sqrt(x*x+y*y+z*z);}
};

struct PYD {
    double pitch, yaw, distance;
    PYD() : pitch(0), yaw(0), distance(0) {}
    PYD(double p, double y, double d) : pitch(p), yaw(y), distance(d) {}
};
//相机坐标系：

struct CXYD {
    double cx, cy, k;//when used as XYV , k > 2 means visible
    CXYD() : cx(0), cy(0), k(0) {}
    CXYD(double cx, double cy, double k) : cx(cx), cy(cy), k(k) {}
    CXYD(double cx, double cy) : cx(cx), cy(cy), k(1) {}
};
struct XYV {
    double x, y;
    bool visible;
    XYV() : x(0), y(0), visible(true) {}
    XYV(double x, double y) : x(x), y(y), visible(true) {}
    XYV(CXYD cxy) : x(cxy.cx), y(cxy.cy), visible(cxy.k > 2) {}
};

// //前向声明
// namespace solver {
//     struct ImuData;
// }


namespace solver {


    /*
    ### 1. 非融合代理类
- XYZProxy ：用于操作非融合的XYZ坐标
- PYDProxy ：用于操作非融合的PYD坐标
- CXYProxy ：用于操作非融合的CXY坐标
### 2. 融合代理类
- XYZIMUProxy ：用于操作融合IMU数据后的XYZ坐标
- PYDIMUProxy ：用于操作融合IMU数据后的PYD坐标
- CXYIMUProxy ：用于操作融合IMU数据后的CXY坐标*/
// 抽象转换接口，由外部实现
class BaseSolver {
public:
    // pyd <-> xyz
    virtual PYD XYZ2PYD(const XYZ &in) const = 0;//留虚函数接口改写。
    virtual XYZ PYD2XYZ(const PYD &in) const = 0;

    // cxy <-> pyd
    virtual XYZ CXYD2XYZ(const CXYD &in) const = 0;
    virtual CXYD XYZ2CXYD(const XYZ &in) const = 0;

    // pyd + imu -> pyd_imu
    virtual XYZ camera2world(const XYZ &p, const PYD &imu) const = 0;
    // pyd_imu + imu -> pyd（即还原非融合的值）
    virtual XYZ world2camera(const XYZ &p_imu, const PYD &imu) const = 0;
};

} // namespace solver


//private: Location &parent_;提供接口接受数据
// Location类：内部以 base_pyd_ 为非融合基本数据，所有非融合数据均由此计算得到。  
// 融合数据通过 camera2world(base_pyd_, imu) 得到，即 fused_pyd_；进而 xyz_imu 与 cxy_imu 由 fused_pyd_ 转换得出。
namespace location {
    class Location {
    public:
        // 注册全局Solver实例
        //控制器里的Solver::registerSolver
        //registerSolver 是一个静态函数，用于注册全局的 solver 实例。
        // solver 是一个实现了 BaseSolver 接口的类，负责各种坐标系之间的转换
        static void registerSolver(const std::shared_ptr<const solver::BaseSolver>& s) {
            solverInstance_ = s;
            solverRegistered_ = (s != nullptr);
        }

        // 非融合代理类：xyz、cxy
        class XYZProxy {
        public:
            XYZProxy(Location &parent) : parent_(parent) {}
            //初始化并构造传递给_parent
            XYZProxy& operator=(const XYZ &value) {
                //把XYZ坐标系的类传递给代理类，并且重载“=”运算符
                if (!solverRegistered_) {
                    LOG(ERROR) << "Solver not registered. XYZ assignment ignored.";
                    return *this;//返回当前对象，不进行复制操作
                }
                parent_.base_xyz_ = value;
                parent_.updateFused();
                return *this;
                //更新之后再返回当前值。
            }
            operator XYZ() const { return parent_.base_xyz_; }
        private:
            Location &parent_;
        };

        class PYDProxy {
        public:
            PYDProxy(Location &parent) : parent_(parent) {}
            PYDProxy& operator=(const PYD &value) {
                if (!solverRegistered_) {
                    LOG(ERROR) << "Solver not registered. PYD assignment ignored.";
                    return *this;
                }
                // 利用 PYD2XYZ 转换后更新基本变量
                parent_.base_xyz_ = solverInstance_->PYD2XYZ(value);
                parent_.updateFused();
                return *this;
            }
            operator PYD() const {
                if (!solverRegistered_) return PYD{0, 0, 0};
                return solverInstance_->XYZ2PYD(parent_.base_xyz_);
            }
        private:
            Location &parent_;
        };

        class CXYProxy {
        public:
            CXYProxy(Location &parent) : parent_(parent) {}
            CXYProxy& operator=(const CXYD &value) {
                if (!solverRegistered_) {
                    LOG(ERROR) << "Solver not registered. CXY assignment ignored.";
                    return *this;
                }
                // 利用 CXYD2XYZ 转换后更新基本变量
                parent_.base_xyz_ = solverInstance_->CXYD2XYZ(value);
                parent_.updateFused();
                return *this;
            }
            operator CXYD() const {
                if (!solverRegistered_) return CXYD{0, 0};
                return solverInstance_->XYZ2CXYD(parent_.base_xyz_);
            }
        private:
            Location &parent_;
        };

        // 融合数据代理类（带 imu 融合）：xyz_imu、cxy_imu
        class XYZIMUProxy {
        public:
            XYZIMUProxy(Location &parent) : parent_(parent) {}
            XYZIMUProxy& operator=(const XYZ &value) {
                if (!solverRegistered_) {
                    LOG(ERROR) << "Solver not registered. Fused XYZ assignment ignored.";
                    return *this;
                }
                // 通过 separateIMU 还原基础 xyz，再更新融合数据
                parent_.base_xyz_ = solverInstance_->world2camera(value, parent_.imu);
                parent_.updateFused();
                return *this;
            }
            operator XYZ() const {
                if (!solverRegistered_) return XYZ{0, 0, 0};
                return solverInstance_->camera2world(parent_.base_xyz_, parent_.imu);
            }
        private:
            Location &parent_;
        };

        class PYDIMUProxy {
        public:
            PYDIMUProxy(Location &parent) : parent_(parent) {}
            PYDIMUProxy& operator=(const PYD &value) {
                if (!solverRegistered_) {
                    LOG(ERROR) << "Solver not registered. Fused PYD assignment ignored.";
                    return *this;
                }
                // 通过 separateIMU 还原基础 xyz，再更新融合数据
                parent_.base_xyz_ = solverInstance_->world2camera(solverInstance_->PYD2XYZ(value), parent_.imu);
                parent_.updateFused();
                return *this;
            }
            operator PYD() const {
                if (!solverRegistered_) return PYD{0, 0, 0};
                return solverInstance_->XYZ2PYD(solverInstance_->camera2world(parent_.base_xyz_, parent_.imu));
            }
        private:
            Location &parent_;
        };

        class CXYIMUProxy {
        public:
            CXYIMUProxy(Location &parent) : parent_(parent) {}
            CXYIMUProxy& operator=(const CXYD &value) {
                if (!solverRegistered_) {
                    LOG(ERROR) << "Solver not registered. Fused CXY assignment ignored.";
                    return *this;
                }
                XYZ tmp = solverInstance_->CXYD2XYZ(value);
                parent_.base_xyz_ = solverInstance_->world2camera(tmp, parent_.imu);
                parent_.updateFused();
                return *this;
            }
            operator CXYD() const {
                if (!solverRegistered_) return CXYD{0, 0};
                return solverInstance_->XYZ2CXYD(solverInstance_->camera2world(parent_.base_xyz_, parent_.imu));
            }
        private:
            Location &parent_;
        };

        // 构造函数初始化所有代理对象
        Location() :
                xyz(*this), cxy(*this), pyd(*this),
                xyz_imu(*this), pyd_imu(*this), cxy_imu(*this) {}
        Location(const Location &other)
                : base_xyz_(other.base_xyz_),
                  fused_xyz_(other.fused_xyz_),
                  imu(other.imu),
                  xyz(*this), cxy(*this), pyd(*this),
                  xyz_imu(*this), pyd_imu(*this), cxy_imu(*this) {}
        Location& operator=(const Location &other) {
            if (this != &other) {
                base_xyz_ = other.base_xyz_;
                fused_xyz_ = other.fused_xyz_;
                imu = other.imu;
                updateFused();
            }
            return *this;
        }

        // 对外公开接口
        XYZProxy xyz;         // 非融合的 xyz
        PYDProxy pyd;
        CXYProxy cxy;         // 非融合的 cxy，从 xyz 转换得出

        XYZIMUProxy xyz_imu;  // 融合后的 xyz
        PYDIMUProxy pyd_imu;
        CXYIMUProxy cxy_imu;  // 融合后的 cxy，从 fused xyz 转换得出

        XYZ getOrgXYZ(const PYD& imu_) const {
            if (!solverRegistered_) {
                LOG(ERROR) << "Solver not registered. getOrgXYZ failed.";
                return XYZ{0, 0, 0};
            }
            return solverInstance_->world2camera(fused_xyz_, imu_);
        }

        PYD getOrgPYD(const PYD& imu_) const {
            if (!solverRegistered_) {
                LOG(ERROR) << "Solver not registered. getOrgPYD failed.";
                return PYD{0, 0, 0};
            }
            return solverInstance_->XYZ2PYD(getOrgXYZ(imu_));
        }

        CXYD getOrgCXY(const PYD& imu_) const {
            if (!solverRegistered_) {
                LOG(ERROR) << "Solver not registered. getOrgCXY failed.";
                return CXYD{0, 0};
            }
            return solverInstance_->XYZ2CXYD(getOrgXYZ(imu_));
        }

        XYZ getImuXYZ(const PYD& imu_) const {
            if (!solverRegistered_) {
                LOG(ERROR) << "Solver not registered. getImuXYZ failed.";
                return XYZ{0, 0, 0};
            }
            return solverInstance_->camera2world(base_xyz_, imu_);
        }

        PYD getImuPYD(const PYD& imu_) const {
            if (!solverRegistered_) {
                LOG(ERROR) << "Solver not registered. getImuPYD failed.";
                return PYD{0, 0, 0};
            }
            return solverInstance_->XYZ2PYD(getImuXYZ(imu_));
        }

        CXYD getImuCXY(const PYD& imu_) const {
            if (!solverRegistered_) {
                LOG(ERROR) << "Solver not registered. getImuCXY failed.";
                return CXYD{0, 0};
            }
            return solverInstance_->XYZ2CXYD(getImuXYZ(imu_));
        }

        PYD imu;  // imu 数据

    private:
        // 更新融合数据
        void updateFused() {
            if (!solverRegistered_) {
                LOG(ERROR) << "Solver not registered. updateFused failed.";
                return;
            }
            fused_xyz_ = solverInstance_->camera2world(base_xyz_, imu);
        }

        // 内部数据存储
        XYZ base_xyz_;  // 非融合的基础 xyz 数据
        XYZ fused_xyz_; // 融合后的 xyz 数据

        // 静态全局变量
        inline static std::shared_ptr<const solver::BaseSolver> solverInstance_;
        inline static bool solverRegistered_;
    };
}