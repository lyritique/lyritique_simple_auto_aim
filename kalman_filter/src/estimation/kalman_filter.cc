# include "estimation/kalman_filter.hpp"
# include <Eigen/src/Core/Matrix.h>
# include <cmath>
# include <eigen3/Eigen/Eigen>
# include <iostream>



/*
EKF追踪X=(px,py,pz,yaw,vx,vy,vz,vyaw,r)
predict X_=f(X),P_=F*P*F^T+Q ,F=f(X)一阶导
新息    h(X_,id)从八维变为四维装甲板数据，y=z-h(X_)，S=H*P_*H^T+R，H=h(X_)一阶导
update  增益K=P*H^T*S^-1，  △X=K*y，  X^+=X_+△X
*/


/* 任务表
1. /update() 传给 associate() 的 z_pred 从来没有被填充
2. /初始化 ObsAssociation 的所有矩阵
3. /第一次 init() 需要明确的初始板 ID。当前用 testResult.id，默认是 -1
4. /P.fill(0.1) 把协方差的非对角项全设成 0.1，不是初始方差。应清零后只设置对角线，例如位置、角度、速度、半径分别设合理方差
5. /reset() 没有清理 F、z_pred 和上一板 ID
6.- 未应用 9 ms offset
7. /id_list 只会存入已经匹配过的 ID。第一次永远只能尝试 ID 0，因此 ID 1/2/3 永远没有机会被选中：kalman_filter.cc (src/kalman_filter.cc:122)。
8. /每次更新都会向 id_list 追加一个 ID，造成重复项无限增长：kalman_filter.cc (src/kalman_filter.cc:280)。
9. /renewArmorList() 把所有条目设为无效，下一个循环又擦除，边擦边遍历，行为不可靠：kalman_filter.cc (src/kalman_filter.cc:296)。
10. /update() 没有检查是否已初始化、匹配是否通过门限。匹配失败仍会继续计算增益：kalman_filter.cc (src/kalman_filter.cc:256)。
11.构造函数只有声明，没有实现；一旦 main 创建 KalmanFilter 就会链接失败。
12.X/P 是私有的，也没有状态读取接口，main 拿不到估计结果。





*/






namespace kalfilter 
{
    namespace 
    {

        constexpr double kPI = 3.14159265358979323846;

        double wrap(double a) 
        {
            while (a > kPI) a -= 2.0 * kPI;
            while (a < -kPI) a += 2.0 * kPI;
            return a;
        }

        //h(X_),是九维列向量转换为四维列向量，从描述中心到描述装甲板
        Eigen::Matrix<double,4,1> observationModel(const Eigen::Matrix<double,9,1>& X_,int id )
        {
            constexpr double kPi = 3.14159265358979323846;
            const double theta = X_(3) + id * kPi * 0.5;
            const double s = std::sin(theta);
            const double c = std::cos(theta);
            const double r = X_(8);

            Eigen::Matrix<double, 4, 1> y;
            y <<X_(0) + r * s,
                X_(1),
                X_(2) + r * c,
                theta;

            return y;
        }

        //计算新息雅克布矩阵H
        Eigen::Matrix<double,4,9> computeH(const Eigen::Matrix<double,9,1>& X_, int id)
        {
            const double c = id * kPI * 0.5;
            const double theta = X_(3) + c;
            const double s = std::sin(theta);
            const double co = std::cos(theta);
            const double r = X_(8);

            Eigen::Matrix<double,4,9> J {Eigen::Matrix<double,4,9>::Zero()};
        

            J(0, 0) = 1.0;
            J(0, 3) = r * co;
            J(0, 8) = s;

            J(1, 1) = 1.0;

            J(2, 2) = 1.0;
            J(2, 3) = -r * s;
            J(2, 8) = co;

            J(3, 3) = 1.0;

            return J;
        }

        Eigen::Matrix4d computeR(const KalmanConfig& config,const cam_ypd ypd)
        {
            //预计算
            const double s_e {std::sin(ypd.ele)};
            const double c_e {std::cos(ypd.ele)};
            const double s_a {std::sin(ypd.azi)};
            const double c_a {std::cos(ypd.azi)};

            /*构造相机系下ypd->直角系的雅可比J
            x = dist · cos(ele) · sin(azi)       右
            y = dist · sin(ele)                  下
            z = dist · cos(ele) · cos(azi)       前（光轴）
            yaw无变换
            */
            Eigen::Matrix4d J {Eigen::Matrix4d::Zero()};
            J.row(0) =  Eigen::Vector4d(ypd.dist*c_e*c_a,  -ypd.dist*s_e*s_a,  c_e*s_a,0.0);      
            J.row(1) =  Eigen::Vector4d(0.0,               ypd.dist*c_e,       s_e    ,0.0);      
            J.row(2) =  Eigen::Vector4d(-ypd.dist*c_e*s_a, -ypd.dist*s_e*c_a,  c_e*c_a,0.0);
            J(3,3) = 1.0;

            //深度噪声
            const double sigma_dist = config.k_dist * ypd.dist;

            //构造R_ypd，理论4*4，但对角，就写成向量吧
            Eigen::Matrix<double,4,1> R_ypd(config.azi*config.azi,config.ele*config.ele,
                                    sigma_dist*sigma_dist,config.yaw*config.yaw);

            return  Eigen::Matrix4d   { J * R_ypd.asDiagonal() * J.transpose()};
            
        }


        ObsAssociation associate 
        (const armerState& armor,
        std::vector<storedID>& id_list,
        const Eigen::Matrix<double,9,1>& X,
        const Eigen::Matrix<double,9,9>& P,
        const KalmanConfig& config,
        const Eigen::Matrix4d& R ,
        int& max_id)
        {
            
            Eigen::Matrix<double,4,1> h_id {};
            Eigen::Matrix<double,4,1> nu {};
            Eigen::Matrix<double,4,9> H_id {};
            Eigen::Matrix4d Sinv_id {};
            double d2 {};

            auto isSameArmor = [&](int id)
            {
                h_id = observationModel(X, id);

                // 欧氏的有关量的计算
                nu = armor.measurement - h_id ;
                nu(3) = wrap(nu(3));

                // 欧氏粗筛
                const double pos = nu.head<3>().norm();
                if (pos > config.assoc_pos_gate) return false;
                if (std::abs(nu(3)) > config.max_yaw_innovation) return false;  // yaw 必须 < 45 度

                // 马氏距离的有关量的计算
                H_id = computeH(X, id);
                Eigen::Matrix4d S = H_id * P * H_id.transpose() + R;
                Sinv_id = S.ldlt().solve(Eigen::Matrix4d::Identity()) ;//ai非说inverse不安全，让我改

                //马氏距离
                d2 = nu.transpose() * Sinv_id * nu;

                //std::cout << "id=" << id
                //<< " d2=" << d2
                //<< " nu_yaw=" << nu(3)
                //<< '\n';

                if (d2 <= config.chi2_gate ) return true;
                else return false;
                
            };

            //再见了id——list
            ObsAssociation best;
            best.d2 = std::numeric_limits<double>::infinity();

            for (int id{};id < 4;++id)
            {
                // 检查这个 id 本帧是否已经使用过
                bool used_this_frame = false;

                for (const auto& stored : id_list)
                {
                    if (stored.id == id && stored.isThisFrame)
                    {
                        used_this_frame = true;
                        break;
                    }
                }

                if (used_this_frame)continue;

                if (!isSameArmor(id)) continue;

                if (d2 < best.d2) 
                {
                best.d2 = d2;
                best.id = id;
                best.nu = nu;
                best.H = H_id;
                best.Sinv = Sinv_id;
                }
                //std::cout << "与第" <<id <<"个板" <<d2 ;
            }
            //std::cout << "\r\n";
          
            
            
            if (best.id >= 0)best.valid = true;


            return best;
            

        }
    }//namespace
    
    KalmanFilter::KalmanFilter(const KalmanConfig& cfg )
    : config(cfg)     //初始化列表
    {
        reset();
        F.setIdentity();
        
    }

    void KalmanFilter::init(armerState& measurement)
    {
        measurement.testResult.id = 0;
        F.setIdentity();

        constexpr double radius = 0.254;
        X.setZero();
        X(8) = radius; //魔法数字
        const double c = measurement.testResult.id * kPI * 0.5;
        X(1) = measurement.measurement(1);                          // 高度/该轴直接对应
        X(3) = measurement.measurement(3);                          // 旋转角，中心yaw零对应首个数据
        //hx模型：q = p - r*n / 
        //因此初始化时：p = q + r*n → 板心 q = 车心 p + r*n / 车心 p = 板心 q - r*n
        X(0) = measurement.measurement(0) - radius * std::sin(measurement.measurement(3));      
        X(2) = measurement.measurement(2) - radius * std::cos(measurement.measurement(3));   

        P.setIdentity();
        const double sp = config.init_sigma_pos * config.init_sigma_pos;
        const double sv = config.init_sigma_vel * config.init_sigma_vel;
        const double sy = config.init_sigma_yaw * config.init_sigma_yaw;
        const double sw = config.init_sigma_vyaw * config.init_sigma_vyaw;
        P(0,0) = sp; P(1,1) = sp; P(2,2) = sp;
        P(3,3) = sy;
        P(4,4) = sv; P(5,5) = sv; P(6,6) = sv;
        P(7,7) = sw;
        P(8,8) = config.kRadiusSigma * config.kRadiusSigma;
        initialized = true;
        time_since_update_ = 0.0;

    }

    
    void KalmanFilter::reset()
    {
        initialized = false;
        X.setZero() ;
        P.setZero() ;
        F.setZero() ;
        id_list.clear() ;
        max_id =0;
        time_since_update_ = 0.0;
    }
   

    void KalmanFilter::predict(const double& dt)
    {
        // 只要还在预测，就说明这一帧没吃到观测。
        time_since_update_ += dt;

        X.head<3>() += X.segment<3>(4)*dt;
        // yaw 是角度，必须折回 ±π，否则自旋时状态会一路漂到 ±nπ，
        // 后面按 id 挑装甲板、算残差都会错位。
        X(3) = wrap(X(7)*dt+X(3));

        //计算雅克布矩阵
        //让它特定位置的4*4块变成单位乘dt，就是根据dt更新了
        F.template block<4,4>(0,4) = dt * Eigen::Matrix4d::Identity();

        //分段白噪声模型制造Q
        //模型中目标的加速度是一个分段常数，在每一段内以为白噪声形式随机变化
        Eigen::Matrix<double, 9, 9> Q {Eigen::Matrix<double,9,9>::Zero()};
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        auto fill_block = [&](int i, int j, double p) 
        {
            Q(i,i) = p * dt4 / 4.0;
            Q(i,j) = p * dt3 / 2.0;
            Q(j,i) = p * dt3 / 2.0;
            Q(j,j) = p * dt2;
        };//lambda看着真舒服，[&]后不用多写输入

        fill_block(0,4, config.p_coord);
        fill_block(1,5, config.p_coord);
        fill_block(2,6, config.p_coord);
        fill_block(3,7, config.p_yaw);
        Q(8,8) = config.p_r * dt; 

        P = F * P * F.transpose() + Q;

        //P(8, 8) *= config.kRadiusSigma;
        
        //正定化，就是沿对角线对称
        P = 0.5 * (P + P.transpose());

    }


    void KalmanFilter::update ( armerState& armor  ,const cam_ypd& ypd)
    {
        /*
        //h(X_)部分写在匿名命名空间
        //计算残差
        Eigen::Matrix<double,4,1> residual {armor.measurement-observationModel(X,armor.testResult.id)};
        residual(3) = wrap(residual(3));

        //计算新息雅克布矩阵H写在匿名命名空间

        //观测噪声协方差矩阵 R 写在匿名命名空间
        //基于上科大开源技术

        Eigen::Matrix<double,4,9> H {computeH(X, armor.testResult.id)};
        Eigen::Matrix4d R {computeR(config, ypd)};

        //计算新息协方差矩阵S
        Eigen::Matrix4d S {H * P * H.transpose() + R};
        */

        //检查初始化
        if (!initialized) return;

        //保存先验
        const Eigen::Matrix<double, 9, 1> x_prior = X;
        const Eigen::Matrix<double, 9, 9> P_prior = P;

        //判断armor的id，顺带算好H，nu（残差）,S^-1
        Eigen::Matrix4d R {computeR(config, ypd)};
        armor.testResult = associate(armor,id_list,X,P,config,R,max_id);

        //检查是否过了匹配
        if (!armor.testResult.valid)return;

        //保存先验，写迭代
        Eigen::Matrix<double, 9, 1> x_iter = x_prior;

        constexpr int max_iterations = 5;

        for (int iteration = 0;iteration < max_iterations;++iteration)
        {
            // 1. 在 x_iter 处重新线性化
            const Eigen::Matrix<double, 4, 9> H =
            computeH(x_iter, armor.testResult.id);

            // 2. 新息协方差
            const Eigen::Matrix4d S  {H * P_prior * H.transpose() + R};

            const Eigen::Matrix4d S_inv {S.ldlt().solve(Eigen::Matrix4d::Identity())};
            
            // 3. 增益    
            const Eigen::Matrix<double, 9, 4> K {P_prior * H.transpose() * S_inv};
   
            // 4. 当前迭代点与观测的差    
            Eigen::Matrix<double, 4, 1> residual = armor.measurement - observationModel(x_iter, armor.testResult.id);   
            residual(3) = wrap(residual(3));

    
            // 5. 当前状态相对先验移动了多少   
            Eigen::Matrix<double, 9, 1> delta = x_iter - x_prior;

    
            // yaw 分量同样需要环绕
            delta(3) = wrap(delta(3));

    
            // 6. IEKF 补偿项
            residual += H * delta;
            residual(3) = wrap(residual(3));

    
            // 7. 从x_prior出发
            Eigen::Matrix<double, 9, 1> x_next = x_prior + K * residual;
            

            // 8. 判断是否收敛
            Eigen::Matrix<double, 9, 1> step = x_next - x_iter;
            step(3) = wrap(step(3));
            x_iter = x_next;

            //每一项都要很小
            const bool converged =
            step.head<3>().norm() < 1e-4 &&
            std::abs(step(3)) < 1e-4 &&
            step.segment<3>(4).norm() < 1e-3 &&
            std::abs(step(7)) < 1e-3 &&
            std::abs(step(8)) < 1e-4;

            if (converged)break;
        }

        //把armor存起来，便于下一帧判断
        storedID armorID ;
        armorID.id = armor.testResult.id;
        armorID.isThisFrame = true;
        id_list.push_back(armorID);
        
        
        //kalman增益K
        Eigen::MatrixXd K {P * armor.testResult.H.transpose()*armor.testResult.Sinv};

        //更新X，P
        X = x_iter;
        // 迭代更新后可能又被推出 ±π，收尾再折一次。
        X(3) = wrap(X(3));

        const Eigen::Matrix<double, 4, 9> H_final =computeH(X, armor.testResult.id);

        const Eigen::Matrix4d S_final =H_final * P_prior * H_final.transpose() + R;

        const Eigen::Matrix4d S_inv_final =S_final.ldlt().solve( Eigen::Matrix4d::Identity());

        const Eigen::Matrix<double, 9, 4> K_final =P_prior * H_final.transpose() * S_inv_final;

        const Eigen::Matrix<double, 9, 9> IKH =Eigen::Matrix<double, 9, 9>::Identity() - K_final * H_final;

        P = IKH * P_prior * IKH.transpose()+ K_final * R * K_final.transpose();

        P = 0.5 * (P + P.transpose());

        // 真正吃到了观测，丢目标计时清零。
        time_since_update_ = 0.0;
    }

    //这个是每一帧的结束处理id_list的
    void KalmanFilter::renewArmorList ()
    {
        if (id_list.empty()) return;
        
        //删除上一帧
        //erase-remove
        id_list.erase(    
            std::remove_if(id_list.begin(), id_list.end(),               
                [](const auto& e) { return !e.isThisFrame; }),   
            id_list.end());
        //这一帧的改为下帧
        for (auto& e :id_list)if (e.isThisFrame)e.isThisFrame=false;
        
    }

    KFResult KalmanFilter::getOutput() const
    {
        KFResult result;

        if (!initialized)
        {
            return result;
        }

        // 太久没观测就别再外推：预测越久越离谱，下游会拿着假状态去转云台。
        if (time_since_update_ > config.lost_timeout)
        {
            return result;
        }

        result.valid = true;
        result.state = X;
        //result.covariance = P;

        return result;
    }


}//namespace kalfilter




