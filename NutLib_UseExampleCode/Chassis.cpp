
#include "../NutLib/Chassis/OmniChassis.hpp"
#include "../NutLib/Chassis/SteerChassis.hpp"
#include "../NutLib/Chassis/SteerChassisSp.hpp"
#include "../NutLib/Controller/DualShock.hpp"
    
using namespace nut;


namespace sample_code{

    /* 最大速度 */
    constexpr float TOP_SPEED = 0.5;//[m/s] 
    constexpr float TOP_ROT = M_PI_4;//[rad/s] 


    void main(){
        /* 各種足周りインスタンス
        * 面倒なのでコンストラクタの記述は省略
        */
        OmniChassis<3> omni{...};           //三輪オムニ
        SteerChassis<3> steer{...};         //三輪ステア
        SteerChassisSp<3> steer_sp{...};    //三輪ステア(ステアドライバ使用)

        /* 説明用コントローラ */
        DualShock dual_shock{...};


        /* 足周りの一元化 */
        Chassis* chassis = &omni;
        //Chassis* chassis = &steer;
        //Chassis* chassis = &steer_sp;

        /* 各種初期化 */
        chassis->Init();
        dual_shock.Init();


        /* 制御ループ的なやつ */
        while(true){
            /* コントローラ入力取得 */
		    Coordinate<float> vec(
                dual_shock.GetButtonData(DualShock::PS3_ANALOG_LX), 
                dual_shock.GetButtonData(DualShock::PS3_ANALOG_LY), 
                0);
            vec *= TOP_SPEED / DualShock::USE_ANAROG_MAX; 
            if(dual_shock.GetButtonData(DualShock::PS3_L1))vec.theta() = -TOP_ROT;
            else if(dual_shock.GetButtonData(DualShock::PS3_R1))vec.theta() = TOP_ROT;

            /* 速度入力 */
            chassis->SetVelocity(vec);
        }
    }
}





/* ステアの特殊入力を使いたい場合 */
namespace sample_code2{

    /* 最大速度 */
    constexpr float TOP_SPEED = 0.5;//[m/s] 
    constexpr float TOP_ROT = M_PI_4;//[rad/s] 


    void main(){
        /* 各種足周りインスタンス
        * 面倒なのでコンストラクタの記述は省略
        */
        SteerChassis<3> steer{...};         //三輪ステア
        SteerChassisSp<3> steer_sp{...};    //三輪ステア(ステアドライバ使用)

        /* 説明用コントローラ */
        DualShock dual_shock{...};


        /* 足周りの一元化 */
        SteerChassisBase* chassis = &steer;
        //SteerChassisBase* chassis = &steer_sp;

        /* 各種初期化 */
        chassis->Init();
        dual_shock.Init();


        /* 制御ループ的なやつ */
        while(true){
            /* コントローラ入力取得 */
		    Coordinate<float> vec(
                dual_shock.GetButtonData(DualShock::PS3_ANALOG_LX), 
                dual_shock.GetButtonData(DualShock::PS3_ANALOG_LY), 
                0);
            vec *= TOP_SPEED / DualShock::USE_ANAROG_MAX; 
            if(dual_shock.GetButtonData(DualShock::PS3_L1))vec.theta() = -TOP_ROT;
            else if(dual_shock.GetButtonData(DualShock::PS3_R1))vec.theta() = TOP_ROT;

            /* 速度入力 */
            if(dual_shock.GetButtonData(DualShock::PS3_START))chassis->SetVelocity({0.0}, SteerChassisBase::MoveMode::reset);
            else if(dual_shock.GetButtonData(DualShock::PS3_SELECT))chassis->SetVelocity({1.0}, SteerChassisBase::MoveMode::steerBreaking);//速度が0だとブレーキされないバグがある(はよなおせ)
            else if(dual_shock.GetButtonData(DualShock::PS3_CIRCLE))chassis->SetVelocity(vec, SteerChassisBase::MoveMode::steerOnry);
            else chassis->SetVelocity(vec);
        }
    }
}