/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>
template <typename T>
KeyBoard<T>::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}
template <typename T>
KeyBoard<T>::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}
template <typename T>
UserCommand KeyBoard<T>::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;     //passive
    case '2':
        return UserCommand::L2_A;     // fixstand
    case '3':
        return UserCommand::L2_X;    //   freestand
    case '4':
        return UserCommand::START;   //   trotting
    case '0':
        return UserCommand::L1_X;    //   balancetest
    case '9':
        return UserCommand::L1_A;    //   swingtest
    case '8':
        return UserCommand::L1_Y;    //  steptest
    case '5':
        return UserCommand::ROLL;    //  wheel
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}
template <typename T>
void KeyBoard<T>::changeValue(){
    switch (_c){
    case 'w':case 'W':
        // std::cout << "up,输入W" << std::endl;
        userValue.ly = min<T>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<T>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<T>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<T>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<T>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<T>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<T>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<T>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}
template <typename T>
void* KeyBoard<T>::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}
template <typename T>
void* KeyBoard<T>::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
            _c = '\0';
        }
        usleep(1000);
    }
    return NULL;
}
// template class KeyBoard<double>;
template class KeyBoard<float>;